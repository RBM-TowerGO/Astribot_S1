from http.server import ThreadingHTTPServer
import json
import threading
from urllib.request import urlopen, Request
from urllib.error import HTTPError
import pytest
from towergo_real.viewer import ViewState, handler_for


def test_readonly_http_and_stale_age():
    state=ViewState();state.update('control',{'state':'disabled'})
    server=ThreadingHTTPServer(('127.0.0.1',0),handler_for(state,b'<html>map</html>'))
    thread=threading.Thread(target=server.serve_forever);thread.start()
    url=f'http://127.0.0.1:{server.server_port}'
    try:
        assert urlopen(url).read()==b'<html>map</html>'
        data=json.load(urlopen(url+'/api/state'))
        assert data['control']['state']=='disabled' and data['ages']['control']>=0
        with pytest.raises(HTTPError) as error:urlopen(Request(url+'/api/state',data=b'{}'))
        assert error.value.code==405
        with pytest.raises(HTTPError) as error:urlopen(url+'/../../etc/passwd')
        assert error.value.code==404
    finally:
        server.shutdown();server.server_close();thread.join()
