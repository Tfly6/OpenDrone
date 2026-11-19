"""Draw bilingual vector Figure 1: modular execution and cross-level evaluation."""
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/opendrone_manuscript_mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.font_manager import FontProperties

HERE = Path(__file__).resolve().parent
INK, TEAL = '#243b4a', '#216b78'


def draw(language='en'):
    (HERE / 'assets').mkdir(parents=True, exist_ok=True)
    zh = language == 'zh'
    font = (FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
            if zh else FontProperties(family='DejaVu Sans'))
    fig, ax = plt.subplots(figsize=(8.4, 3.85))
    fig.subplots_adjust(left=.015, right=.985, top=.99, bottom=.015)
    ax.set(xlim=(0, 16), ylim=(0, 7.1)); ax.axis('off')

    def text(x, y, value, size=11, bold=False, color=INK):
        ax.text(x, y, value, fontsize=size, fontproperties=font,
                weight='bold' if bold else 'normal', color=color,
                ha='center', va='center', linespacing=1.3)

    def box(x, y, w, h, value, fill='#f3f6f8', edge=INK, bold=False):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle='round,pad=0.015,rounding_size=0.09',
                     facecolor=fill, edgecolor=edge, lw=.9, zorder=3))
        text(x+w/2, y+h/2, value, size=10 if w<3 else 11, bold=bold)

    def arrow(points, color=INK, dashed=False):
        if len(points)>2:
            xs, ys = zip(*points[:-1])
            ax.plot(xs, ys, color=color, lw=1.15,
                    linestyle='--' if dashed else '-', zorder=1)
        ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle='-|>',
                     mutation_scale=10, color=color, lw=1.15,
                     linestyle='--' if dashed else '-', zorder=2))

    text(8, 6.8, '闭环执行' if zh else 'Closed-loop execution', 11.5, True)
    for y, label in zip([5.55, 4.65, 3.75],
                        ['解析参考', '轨迹生成器', '在线规划器'] if zh else
                        ['Analytic reference', 'Trajectory generator', 'Online planner']):
        box(.2, y, 2.85, .67, label)
        ax.plot([3.05, 3.42], [y+.335, y+.335], color=INK, lw=1.1)
    ax.plot([3.42, 3.42], [4.085, 5.885], color=INK, lw=1.1)
    arrow([(3.42, 4.985), (4.05, 4.985)])
    box(4.08, 4.2, 2.65, 1.57,
        '时间参考\n适配接口' if zh else 'Timed-reference\ninterface')
    box(7.5, 4.2, 2.65, 1.57, '控制器' if zh else 'Controller')
    box(10.95, 4.2, 4.85, 1.57, 'PX4 + Gazebo', fill='#e8f0f2', bold=True)
    arrow([(6.76, 4.985), (7.45, 4.985)])
    arrow([(10.18, 4.985), (10.9, 4.985)])

    arrow([(13.35, 4.18), (13.35, 2.98), (1.42, 2.98), (1.42, 3.72)], TEAL)
    arrow([(8.82, 2.98), (8.82, 4.17)], TEAL)
    text(7.7, 2.65, '状态反馈' if zh else 'State feedback', 10, color=TEAL)

    # Record native progress, timed reference and measured execution.
    for x, y in [(2.65, 3.72), (5.40, 4.18), (14.90, 4.18)]:
        arrow([(x, y), (x, 2.12)], '#7f8d95', True)
    box(.2, 1.37, 15.6, .72, '跨层级评价' if zh else 'Cross-level evaluation',
        fill='#e8f0f2', edge=TEAL, bold=True)
    labels = ['轨迹性质', '跟踪误差', '任务结果', '参考覆盖率'] if zh else [
        'Trajectory metrics', 'Tracking error', 'Task outcome', 'Reference coverage']
    for x, label in zip([.2, 4.2, 8.2, 12.2], labels):
        arrow([(x+1.8, 1.35), (x+1.8, 1.0)], TEAL)
        box(x, .23, 3.6, .73, label, fill='white', edge='#b8c7cd')

    name = 'evidence_flow_zh' if zh else 'evidence_flow'
    fig.savefig(HERE/'assets'/f'{name}.pdf')
    fig.savefig(HERE/'assets'/f'{name}.png', dpi=200)
    plt.close(fig)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE / 'outputs/framework')
    HERE = parser.parse_args().output.expanduser().resolve()
    draw('en'); draw('zh')
