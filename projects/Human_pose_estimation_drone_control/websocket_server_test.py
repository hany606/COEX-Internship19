#! /usr/bin/python
# This script was forked from https://www.hackster.io/innat/raspberry-pi-websocket-acef41

import os.path
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web


#Tornado Folder Paths
settings = dict(
	template_path = os.path.join(os.path.dirname(__file__), "templates"),
	static_path = os.path.join(os.path.dirname(__file__), "static")
	)

#Tonado server port
PORT = 8091

class WSHandler(tornado.websocket.WebSocketHandler):
  def open(self):
    print ('[WS] Connection was opened.')
 
  def on_message(self, message):
    print (('[WS] Incoming message:'), message)
  def on_close(self):
    print ('[WS] Connection was closed.')
  def check_origin(self,origin):
    return True

application = tornado.web.Application([(r'/ws', WSHandler)], **settings)


if __name__ == "__main__":
    try:
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(PORT)
        main_loop = tornado.ioloop.IOLoop.instance()

        print ("Tornado Server started")
        main_loop.start()

    except:
        print ("Exception triggered - Tornado Server stopped.")

