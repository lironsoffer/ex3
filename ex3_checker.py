from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import xmlrpc.client
import socket
from multiprocessing import Process
import ex3
import traceback
import sys

def convert_dictionary_to_string_keys(d):
    newd = {}
    for k in d:
        newd[repr(k)] = d[k]
    return newd

def convert_dictionary_from_string_keys(d):
    newd = {}
    for k in d:
        newd[tuple(eval(k))] = d[k]
    return newd


def check_solution(problem, state):
    controller = ex3.SpaceshipController(problem, state)

    hostname = socket.gethostname()
    #print("hostname", hostname)
    ipaddress = socket.gethostbyname(socket.gethostname())
    #print("IP address", ipaddress)

    # Restrict to a particular path.
    class RequestHandler(SimpleXMLRPCRequestHandler):
        rpc_paths = ('/RPC2',)
    
    server = SimpleXMLRPCServer(("0.0.0.0", 40031),
                                requestHandler=RequestHandler, allow_none=True)
    
    done = False
    
    def ex3_choose_next_action(state):
        #print("got state", state)
        state_tuple = tuple([tuple(state[0]), state[1], convert_dictionary_from_string_keys(state[2]), state[3], state[4], state[5][0]])
        try:
            action = controller.choose_next_action(state_tuple)
            #print("returning action", action)        
            return action
        except:
            traceback.print_exc()
            return ""

    
    def printfunc(*message):
        print("from server: ", *message)
        
    def finished():
        #print("finished")
        server.server_close()
        done = True
                    
    # Create server
    def serve_policy():
        #print("creating query policy server")
        server.register_function(ex3_choose_next_action, 'ex3_choose_next_action')
        server.register_function(printfunc, 'printfunc')
        server.register_function(finished, 'finished')        
        #print("starting query policy server")   
        try:
            while not done:                        
                server.handle_request()            
        except Exception as e:
            pass
            #print(e)
        #print("stopping query policy server")
        
        
        
    
    
    
    p = Process(target=serve_policy)
    p.start()   
    
    s = xmlrpc.client.ServerProxy('http://172.17.0.1:40003', allow_none=True)
    #s = SimpleXMLRPCServer(("127.0.0.1",40002), allow_none=True)
    #s = xmlrpc.client.ServerProxy('http://127.0.0.1:40003', allow_none=True)
    
    
    
    s.handle_ex3(ipaddress, problem[0:5] + (convert_dictionary_to_string_keys(problem[5]), ) + problem[6:])
    p.join()
    
    
    

