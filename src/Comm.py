"""
        A library containing the interfaces to communicate with an SQL knowledge base or with other devices via TCP/IP
        When the simulation software want to be used in an experimental setup, it is sufficient to change the methods
        in this library.
"""


def tcp_write(destination, data):
    destination.put(data)


def sql_write(destination, data):
    destination.put(data)


def sql_read(request):
    return request.items


def sql_remove(request, order_number):
    request.get(lambda task: task.order_number == order_number)
