import SimpleHTTPServer
import SocketServer
from urlparse import urlparse, parse_qsl

PORT = 8080
FRONT_FILE = "/main.html"
SEARCH_FILE = "/search.html"


class CustomHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):

        # Root path for the front page with a simple menu and instructions.
        if self.path == '/':
            self.path = FRONT_FILE
        else:
            form_url = urlparse(self.path)
            url_params= dict(parse_qsl(form_url.query))
            try:
                # Define next page -- open search page only if authenticated.
                self.path = SEARCH_FILE if self.__authenticate(url_params) else FRONT_FILE

                if self.path.startswith('/search'):
                    self.path = SEARCH_FILE

            except KeyError:
                self.path = FRONT_FILE

        return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)


    def __authenticate(self, log_parameters):
        """ Method for user authentication by name & password."""
        user_name = log_parameters['user_name']
        password = log_parameters['password']
        print("There will be checking of {} and {}".format(user_name, password))
        return True


Handler = CustomHandler
httpd = SocketServer.TCPServer(("127.0.0.1", PORT), Handler)
print("serving at port", PORT)
httpd.serve_forever()
