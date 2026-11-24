"""Readable region labels derived only from the model's actual semantic output."""
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PMF_LABELS = {
    'unlabeled':'忽略', 'car':'汽车', 'bicycle':'自行车', 'motorcycle':'摩托车',
    'truck':'卡车', 'other-vehicle':'其他车辆', 'person':'人', 'bicyclist':'骑车人',
    'motorcyclist':'摩托骑手', 'road':'道路', 'parking':'停车区', 'sidewalk':'人行道',
    'other-ground':'其他地面', 'building':'建筑', 'fence':'围栏', 'vegetation':'植被',
    'trunk':'树干', 'terrain':'地形', 'pole':'杆', 'traffic-sign':'交通标志',
}
DISPLAY_LABELS = {**PMF_LABELS,
    'background':'背景', 'accordion':'手风琴', 'airplane':'飞机', 'axe':'斧头',
    'baby':'婴儿', 'bassoon':'巴松管', 'bell':'铃', 'bird':'鸟', 'boat':'船',
    'boy':'男孩', 'bus':'公交车', 'cat':'猫', 'cello':'大提琴', 'clarinet':'单簧管',
    'clipper':'理发器', 'clock':'钟', 'dog':'狗', 'donkey':'驴', 'drum':'鼓',
    'duck':'鸭', 'elephant':'象', 'emergency-car':'应急车辆', 'erhu':'二胡',
    'flute':'长笛', 'frying-food':'煎炸食物', 'girl':'女孩', 'goose':'鹅',
    'guitar':'吉他', 'gun':'枪', 'guzheng':'古筝', 'hair-dryer':'吹风机',
    'handpan':'手碟', 'harmonica':'口琴', 'harp':'竖琴', 'helicopter':'直升机',
    'hen':'母鸡', 'horse':'马', 'keyboard':'电子琴', 'leopard':'豹', 'lion':'狮',
    'man':'男人', 'marimba':'马林巴琴', 'missile-rocket':'导弹／火箭',
    'mower':'割草机', 'parrot':'鹦鹉', 'piano':'钢琴', 'pig':'猪', 'pipa':'琵琶',
    'saw':'锯', 'saxophone':'萨克斯', 'sheep':'羊', 'sitar':'西塔琴',
    'sorna':'唢呐', 'squirrel':'松鼠', 'tabla':'塔布拉鼓', 'tank':'坦克',
    'tiger':'虎', 'tractor':'拖拉机', 'train':'火车', 'trombone':'长号',
    'trumpet':'小号', 'tuba':'大号', 'ukulele':'尤克里里', 'utv':'多用途车辆',
    'vacuum-cleaner':'吸尘器', 'violin':'小提琴', 'wolf':'狼', 'woman':'女人',
}

FONT_PATHS = (
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
)


def supports_annotation_text(candidate):
    missing = candidate.getmask('\U0010ffff')
    signature = (missing.size, bytes(missing))
    return all((mask.size, bytes(mask)) != signature
               for mask in (candidate.getmask(char) for char in 'RGB123+·墙模型预测'))


@lru_cache(maxsize=4)
def font(size):
    for path in FONT_PATHS:
        if Path(path).exists():
            candidate = ImageFont.truetype(path, size)
            if supports_annotation_text(candidate):
                return candidate
    raise RuntimeError('预测图需要同时支持中文、英文和数字的字体，请安装 fonts-noto-cjk')


def regions(labels, names, valid=None, background=0, limit=5):
    labels = np.asarray(labels)
    sparse = valid is not None
    valid = labels != background if valid is None else np.asarray(valid, bool) & (labels != background)
    if labels.ndim != 2 or valid.shape != labels.shape:
        raise ValueError('Annotation mask and labels must share image dimensions')
    counts = np.bincount(labels[valid], minlength=len(names))
    found = []
    minimum = 8 if sparse else max(32, int(labels.size * .002))
    for class_id in np.argsort(-counts):
        if counts[class_id] < minimum:
            continue
        support = valid & (labels == class_id)
        connected = cv2.dilate(support.astype(np.uint8), np.ones((7,7), np.uint8)) if sparse else support.astype(np.uint8)
        n, components, stats, _ = cv2.connectedComponentsWithStats(connected, connectivity=8)
        for i in range(1,n):
            # Dilation groups sparse projections for display; count original evidence only.
            count = int(np.count_nonzero(support & (components == i)))
            if count < minimum:
                continue
            x,y,w,h = map(int,stats[i,:4])
            contour_mask = (components[y:y+h,x:x+w] == i).astype(np.uint8)
            contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c + np.array([[[x,y]]],np.int32) for c in contours]
            label = str(names[class_id])
            found.append(dict(id=int(class_id),label=label,display_label=DISPLAY_LABELS.get(label,label),
                              bbox=[x,y,w,h],count=count,unit='投影点' if sparse else '像素',
                              contours=contours))
    return sorted(found,key=lambda r:-r['count'])[:limit]


def annotate(rgb, labels, names, colors, valid=None, task='pmf'):
    """Return an annotated RGB image and region metadata; labels stay untouched."""
    selected = regions(labels,names,valid)
    canvas = np.asarray(rgb,np.uint8).copy()
    for row in selected:
        color = tuple(int(v) for v in colors[row['id']])
        cv2.drawContours(canvas,row['contours'],-1,(255,255,255),3)
        cv2.drawContours(canvas,row['contours'],-1,color,1)
        x,y,w,h = row['bbox']
        cv2.rectangle(canvas,(x,y),(x+w-1,y+h-1),color,1)
    picture = Image.fromarray(canvas)
    draw = ImageDraw.Draw(picture)
    width,height = picture.size
    heading = 'RGB + 音频 · 模型预测' if task=='avs' else 'RGB-D + LiDAR · 模型预测' if task=='indoor_rgbd' else 'LiDAR几何 · 移除RGB输入' if task=='indoor_ablation' else '室内 RGB + LiDAR · 模型预测' if task=='indoor' else 'RGB + LiDAR · 模型预测' if task=='pmf' else 'RGB分支 · 模型预测'
    draw.rectangle((0,0,width,23),fill=(12,24,35))
    draw.text((7,1),heading,font=font(15),fill=(232,244,251))
    for number,row in enumerate(selected,1):
        x,y,_,_ = row['bbox']
        draw.text((x+3,min(height-23,max(26,y+3))),str(number),font=font(14),
                  fill=(255,255,255),stroke_width=2,stroke_fill=(12,24,35))
    occupied = []
    callouts = []
    for number,row in enumerate(selected,1):
        x,y,w,h = row['bbox']
        caption=f'{number} {row["display_label"]} · {row["count"]}{row["unit"]}'
        box = draw.textbbox((0,0),caption,font=font(14))
        bw,bh = min(width-8,box[2]-box[0]+12),25
        left = min(max(4,x),width-bw-4)
        top = min(max(26,y-bh),height-bh-4)
        # Avoid covering another label; prefer space near the corresponding region.
        candidates = [top] + list(range(27,height-bh,27))
        for candidate in candidates:
            rect=(left,candidate,left+bw,candidate+bh)
            if not any(rect[0]<r[2] and rect[2]>r[0] and rect[1]<r[3] and rect[3]>r[1] for r in occupied):
                top=candidate
                break
        occupied.append((left,top,left+bw,top+bh))
        color=tuple(int(v) for v in colors[row['id']])
        callouts.append((left,top,bw,bh,caption,color,x+w/2,max(26,y+h/2)))
    # Draw all lines first so later labels never overwrite an earlier caption.
    for left,top,bw,bh,caption,color,target_x,target_y in callouts:
        draw.line((left+bw/2,top+bh,target_x,target_y),fill=(255,255,255),width=1)
    for left,top,bw,bh,caption,color,_,_ in callouts:
        draw.rounded_rectangle((left,top,left+bw,top+bh),radius=3,fill=(12,24,35),outline=color,width=2)
        draw.text((left+6,top+1),caption,font=font(14),fill=(255,255,255))
    if not selected:
        message='本段未检出可标注的发声区域' if task=='avs' else '本帧没有足够的非忽略预测区域'
        draw.rectangle((0,height-29,width,height),fill=(12,24,35))
        draw.text((7,height-26),message,font=font(15),fill=(238,204,146))
    metadata=[{k:v for k,v in row.items() if k!='contours'} for row in selected]
    return np.asarray(picture),metadata
