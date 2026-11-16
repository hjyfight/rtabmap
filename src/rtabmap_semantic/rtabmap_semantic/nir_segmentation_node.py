"""Independent NIR YOLO baseline. No RGB pixels, masks or labels enter inference."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from std_msgs.msg import String
from rtabmap_semantic_msgs.msg import SegmentationImageRequest, SegmentationFrame
from .detector import YoloDetector
from .segmentation import vocabulary,prepare_instances
from .segmentation_messages import frame_message


class NirSegmentationNode(Node):
    def __init__(self,detector=None,**kwargs):
        super().__init__('nir_segmentation',**kwargs)
        defaults=dict(model_path='models/yoloe-11s-seg-pf.pt',confidence=.45,image_size=640,cpu_threads=2,device='cpu')
        c={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        self.bridge=CvBridge()
        self.detector=detector or YoloDetector(c['model_path'],confidence=c['confidence'],image_size=c['image_size'],
                                             threads=c['cpu_threads'],backend='yoloe_pf',device=c['device'])
        self.labels,self.vocabulary_version=vocabulary(self.detector)
        self.model_sha=hashlib.sha256(Path(c['model_path']).read_bytes()).hexdigest()
        self.provenance=json.dumps(dict(input_modality='nir',nir_domain_validated=False,**c),sort_keys=True)
        self.publisher=self.create_publisher(SegmentationFrame,'/perception/segmentation/nir_baseline',2)
        self.status_pub=self.create_publisher(String,'/miti/segmentation/nir_status',2)
        self.pending=self.future=None
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.completed=0;self.error=''
        self.create_subscription(SegmentationImageRequest,'/perception/segmentation/nir_request',self.receive,1)
        self.create_timer(.025,self.poll)
        self.create_timer(1.,self.status)

    def receive(self,message):
        self.pending=message

    def infer(self,message):
        if message.image.encoding!='mono8':raise ValueError('NIR inference requires real mono8 input')
        start=time.monotonic()
        gray=self.bridge.imgmsg_to_cv2(message.image,desired_encoding='mono8')
        rows=self.detector.detect(cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR))
        record=dict(header=message.image.header,session_id=message.session_id,sequence=message.sequence,
            revision=0,width=gray.shape[1],height=gray.shape[0],model_sha256=self.model_sha,
            vocabulary_version=self.vocabulary_version,provenance_json=self.provenance,
            result_source='nir_yolo',selection_reason='independent_nir_baseline',provider_id='',
            inference_seconds=time.monotonic()-start,instances=prepare_instances(rows,gray.shape,self.labels))
        return frame_message(record,self.bridge)

    def poll(self):
        if self.future is not None and self.future.done():
            future,self.future=self.future,None
            try:self.publisher.publish(future.result());self.completed+=1;self.error=''
            except Exception as error:self.error=str(error);self.get_logger().error(self.error)
        if self.future is None and self.pending is not None:
            message,self.pending=self.pending,None
            self.future=self.pool.submit(self.infer,message)

    def status(self):
        self.status_pub.publish(String(data=json.dumps(dict(schema=1,state='error' if self.error else
            'inferencing' if self.future else 'ready',completed=self.completed,last_error=self.error))))

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init();node=None
    try:node=NirSegmentationNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
