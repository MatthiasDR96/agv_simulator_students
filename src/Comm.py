class Comm:
    """
        A library containing the interfaces to communicate with an SQL knowledge base or with other devices via TCP/IP
        When the simulation software would to be used in an experimental setup, it is sufficient to change the methods
        in this library.
    """
    
    def __init__(self, ip):
        self.ip = ip
    
    def tcp_write(self, destination, data):
        
        # Get destination ip and port
        src_ip = self.ip
        dest_ip = None
        dest_port = None
        
        # Construct JSON datatype
        message = None
        
        # Send message over TCP/IP
        destination.put(data)
    
    def sql_write(self, destination, data):
        
        # Get destination ip and port
        src_ip = self.ip
        dest_ip = None
        dest_port = None
    
        # Construct SQL command
        message = None
    
        # Execute SQL command
        destination.put(data)
    
    def sql_read(self, request):
        
        # Get destination ip and port
        src_ip = self.ip
        dest_ip = None
        dest_port = None
    
        # Construct SQL command
        message = None
    
        # Execute SQL command
        return request.items
    
    def sql_remove(self, request, order_number):
        
        # Get destination ip and port
        src_ip = self.ip
        dest_ip = None
        dest_port = None
    
        # Construct SQL command
        message = None
    
        # Execute SQL command
        request.get(lambda task: task.order_number == order_number)
