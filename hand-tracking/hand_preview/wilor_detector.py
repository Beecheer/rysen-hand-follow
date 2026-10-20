"""WiLoR camera adapter. Local inference only, no SDK calls."""
import http.client
import json
import cv2
from .comparison import landmark_result


class WiLoRDetector:
    def __init__(self):self.connection=http.client.HTTPConnection('127.0.0.1',8093,timeout=3)
    def __enter__(self):return self
    def __exit__(self,*args):self.connection.close()
    def detect_for_video(self,image,timestamp):
        rgb=image.numpy_view()
        ok,jpeg=cv2.imencode('.jpg',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,95])
        if not ok:raise ValueError('WiLoR 图像编码失败')
        try:
            self.connection.request('POST','/infer',jpeg.tobytes(),{'Content-Type':'image/jpeg'})
            response=self.connection.getresponse();data=json.loads(response.read())
            if response.status!=200:raise ValueError('WiLoR 后端：'+str(data.get('detail',data)))
            return landmark_result(data['hands'])
        except Exception as exc:
            self.connection.close()
            result=landmark_result([]);result.backend_error='WiLoR 推理通信异常：'+str(exc)
            return result
