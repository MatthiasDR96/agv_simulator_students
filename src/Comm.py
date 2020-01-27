class Comm:
    """
        A library containing the interfaces to communicate with an SQL knowledge base or with other devices via TCP/IP
        When the simulation software want to be used in an experimental setup, it is sufficient to change the methods
        in this library.
    """
    
    def __init__(self):
        self.agv = None
    
    def tcp_write(self, destination, data):
        destination.put(data)
    
    def sql_write(self, destination, data):
        destination.put(data)
    
    def sql_read(self, request):
        return request.items
    
    def sql_remove(self, request, order_number):
        request.get(lambda task: task.order_number == order_number)
