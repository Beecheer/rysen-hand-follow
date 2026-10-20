"""Persistent localhost transport; never replay a failed control request."""
from http.client import HTTPConnection
import json
import time


class BridgeClient:
    def __init__(self,timeout=.3,port=8092):
        self.timeout=timeout;self.port=port;self.connection=None;self.used_at=0

    def close(self):
        if self.connection:self.connection.close()
        self.connection=None

    def request(self,method,path,body=None,headers=None):
        if self.connection is not None and time.monotonic()-self.used_at>.75:self.close()
        if self.connection is None:self.connection=HTTPConnection('127.0.0.1',self.port,timeout=self.timeout)
        try:
            self.connection.request(method,path,body=body,headers=headers or {})
            response=self.connection.getresponse();payload=response.read(65536)
            if not response.isclosed():raise ValueError('Bridge response too large')
            self.used_at=time.monotonic()
            data=json.loads(payload)
            if response.status!=200:raise ValueError('Local bridge HTTP '+str(response.status)+': '+str(data.get('error','request rejected')))
            return data
        except Exception:
            self.close()
            raise
