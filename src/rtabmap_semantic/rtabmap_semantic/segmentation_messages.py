from rtabmap_semantic_msgs.msg import SegmentationFrame, SegmentationInstance
from .segmentation import dense_maps


def frame_message(record, bridge):
    message=SegmentationFrame()
    for name in ('header','session_id','sequence','revision','width','height','model_sha256',
                 'vocabulary_version','provenance_json','result_source','selection_reason','provider_id','inference_seconds'):
        setattr(message,name,record[name])
    for row in record['instances']:
        instance=SegmentationInstance()
        for name in ('instance_id','class_id','label','confidence','classification_score','bbox_xyxy','mask_offset'):
            setattr(instance,name,row[name])
        instance.mask=bridge.cv2_to_imgmsg(row['mask'],encoding='mono8')
        instance.mask.header=message.header
        message.instances.append(instance)
    classes,identities=dense_maps(record['instances'],(record['height'],record['width']))
    message.class_map=bridge.cv2_to_imgmsg(classes,encoding='32SC1')
    message.instance_map=bridge.cv2_to_imgmsg(identities,encoding='32SC1')
    message.class_map.header=message.instance_map.header=message.header
    return message


def read_instances(message,bridge):
    return [dict(instance_id=r.instance_id,class_id=r.class_id,label=r.label,confidence=r.confidence,
        classification_score=r.classification_score,bbox_xyxy=[float(v) for v in r.bbox_xyxy],mask_offset=[int(v) for v in r.mask_offset],
        mask=bridge.imgmsg_to_cv2(r.mask,desired_encoding='mono8').copy()) for r in message.instances]
