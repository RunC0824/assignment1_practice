#!/usr/bin/env python3
"""Build the one-page English report from the final measured metrics.

Requires reportlab. Result plots and numerical metrics are generated separately
by compare_noise.py using standard Python and Matplotlib.
"""
import argparse
import json
from pathlib import Path
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, KeepTogether

ROOT=Path(__file__).resolve().parents[1]
FONT_DIR=Path('/usr/share/fonts/truetype/dejavu')
for name, filename in [('DejaVuSans','DejaVuSans.ttf'),('DejaVuSans-Bold','DejaVuSans-Bold.ttf'),('DejaVuSansMono','DejaVuSansMono.ttf')]:
    pdfmetrics.registerFont(TTFont(name, str(FONT_DIR/filename)))


def build(metrics_path, output):
    measured=json.loads(metrics_path.read_text())
    base=measured['metrics']['baseline']; noisy=measured['metrics']['noisy']
    window=measured['evaluation_window']
    blue=colors.HexColor('#174e70'); ink=colors.HexColor('#202c39')
    body=ParagraphStyle('Body',fontName='DejaVuSans',fontSize=10,leading=13.8,textColor=ink,spaceAfter=8)
    small=ParagraphStyle('Small',parent=body,fontSize=8.5,leading=11,spaceAfter=5)
    title=ParagraphStyle('Title',parent=body,fontName='DejaVuSans-Bold',fontSize=19,leading=22,spaceAfter=5,textColor=blue)
    section=ParagraphStyle('Section',parent=body,fontName='DejaVuSans-Bold',fontSize=11.2,leading=14,spaceBefore=9,spaceAfter=6,textColor=blue)
    equation=ParagraphStyle('Equation',parent=body,fontName='DejaVuSansMono',fontSize=9.2,leading=12,spaceAfter=3)
    flow=[]
    flow.append(Paragraph('Wheel Odometry and Error Tracking',title))
    flow.append(Paragraph('Assignment 1: Localization and Simulation Foundations | Keran Chen',small))
    flow.append(Paragraph('Ubuntu 24.04 | ROS 2 Jazzy | Gazebo Sim 8.15.0',small))
    flow.append(Spacer(1,4))
    flow.append(Paragraph('Problem Statement',section))
    flow.append(Paragraph('Track the planar pose (x, y, heading) of a differential-drive robot in an indoor simulated room. '
        'The robot executes four nominal 2 m straight sections and four 90-degree turns. '
        'The aim is to measure accumulated wheel-odometry error and isolate the effect of added encoder noise.',body))
    flow.append(Paragraph('Methods',section))
    flow.append(Paragraph('The estimator uses measured wheel-joint angles only, starting from the known configured pose (0, 0, 0). '
        'Angles are quantised to 2,048 counts/revolution and converted to wheel travel using radius r = 0.15 m; '
        'wheel separation b = 0.625 m. The Lecture 2 short-step midpoint update runs at approximately 50 Hz:',body))
    equations=['ds = (dsR + dsL)/2;   dtheta = (dsR - dsL)/b',
               'x_next = x + ds*cos(theta + dtheta/2)',
               'y_next = y + ds*sin(theta + dtheta/2)',
               'theta_next = wrap(theta + dtheta)']
    formula=Table([[Paragraph(line,equation)] for line in equations],colWidths=[495])
    formula.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,-1),colors.HexColor('#f2f6f8')),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),2),('BOTTOMPADDING',(0,0),(-1,-1),2)]))
    flow.append(formula); flow.append(Spacer(1,7))
    flow.append(Paragraph('Wheel measurements follow d_measured = (1 + bias)d + epsilon, with independent Gaussian '
        'epsilon of standard deviation k sqrt(|d|). Left/right scale biases are 0%/1%. '
        'These are declared experimental assumptions; k has units m/sqrt(m). '
        'The same saved encoder trajectory is replayed twice: baseline k = 0 and noisy k = 0.01, with all other settings fixed and seed 42.',body))
    flow.append(Paragraph('Simulator pose is used only for evaluation. Both estimates are generated before ground truth is read. '
        'Truth is interpolated to estimate timestamps in the same world frame; heading interpolation and differences use the shortest angle. '
        'Evaluation starts at first encoder motion and ends one second after last motion, excluding the extended idle tail.',body))
    flow.append(Paragraph('Results',section))
    table=[['Metric','Baseline (k = 0)','Noisy (k = 0.01)'],
           ['Final position error (m)',f'{base["final_position_error_m"]:.4f}',f'{noisy["final_position_error_m"]:.4f}'],
           ['Position RMSE (m)',f'{base["position_rmse_m"]:.4f}',f'{noisy["position_rmse_m"]:.4f}'],
           ['Final signed heading error (deg)',f'{base["final_heading_error_deg"]:.3f}',f'{noisy["final_heading_error_deg"]:.3f}']]
    tab=Table(table,colWidths=[255,120,120],hAlign='LEFT')
    tab.setStyle(TableStyle([('FONTNAME',(0,0),(-1,0),'DejaVuSans-Bold'),('FONTNAME',(0,1),(-1,-1),'DejaVuSans'),
        ('FONTSIZE',(0,0),(-1,-1),9),('TEXTCOLOR',(0,0),(-1,0),colors.white),('BACKGROUND',(0,0),(-1,0),blue),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.HexColor('#f2f6f8'),colors.white]),
        ('ALIGN',(1,0),(-1,-1),'CENTER'),('VALIGN',(0,0),(-1,-1),'MIDDLE'),
        ('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7)]))
    flow.append(tab); flow.append(Spacer(1,8))
    difference=noisy['final_position_error_m']-base['final_position_error_m']
    direction='increased' if difference>=0 else 'decreased'
    flow.append(Paragraph(f'Over {base["duration_s"]:.2f} s and {base["samples"]:,} matched samples per condition, '
        f'added noise {direction} final position error by {abs(difference):.4f} m for this realisation. '
        'The baseline already drifts because of the fixed wheel-scale mismatch and wheel/ground motion differences. '
        'Heading errors rotate subsequent displacement estimates, producing growing trajectory separation. '
        'Random noise need not worsen every endpoint; one route and one seed do not establish a statistical trend.',body))
    flow.append(Paragraph('Evidence: results/trajectory_comparison.png (labelled truth and estimate trajectories), '
        'results/error_comparison.png, and results/metrics.json. Reproduce with python3 scripts/compare_noise.py; '
        'compressed encoder and ground-truth inputs are included. Local Git history records simulation, estimation and evaluation development.',small))
    output.parent.mkdir(parents=True,exist_ok=True)
    doc=SimpleDocTemplate(str(output),pagesize=A4,rightMargin=50,leftMargin=50,topMargin=35,bottomMargin=30,
        title='Assignment 1 - Wheel Odometry and Error Tracking',author='Keran Chen')
    doc.build(flow)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metrics',type=Path,default=ROOT/'results/metrics.json')
    parser.add_argument('--output',type=Path,default=ROOT/'results/Assignment1_Report_Keran_Chen.pdf')
    args=parser.parse_args(); build(args.metrics,args.output)
