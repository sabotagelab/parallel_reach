
#local imports
from F1Hylaa import F1Hylaa

#python lib imports
import sys
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
from functools import partial

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

#started by caller
def startServer(host, port):
    # Create server
    with SimpleXMLRPCServer(
        (host, port),
        requestHandler=RequestHandler
    ) as server:
        server.register_introspection_functions()

        hylaa = F1Hylaa()
        hylaa.set_model_params([0, 0, 0],[0, 0, 0, 0])

        #register the functions for hylaa
        server.register_instance(hylaa)

        server.serve_forever()

if __name__ == "__main__":
    print(sys.argv)
    try:
        port = int(sys.argv[2])
    except Exception as e:
        print("hylaa_server could not be started.")
        print("\twere the arguments valid?")
        print("ERROR: \n\t {}".format(e))

    startServer(sys.argv[1], port)