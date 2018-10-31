import SimpleHTTPServer
import SocketServer
import os

PORT = 8080
FRONT_FILE = "/main.html"
SEARCH_FILE = "/search.html"

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

class CustomHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):

        # Root path for the front page with a simple menu and instructions.
        if self.path == '/':
            self.path = FRONT_FILE

        # Search path for page with form to search for a person (worker).
        if self.path == '/search':
            if self.__authorised():
                self.path = SEARCH_FILE

        return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

Handler = CustomHandler
httpd = SocketServer.TCPServer(("127.0.0.1", PORT), Handler)
print("serving at port", PORT)
httpd.serve_forever()
