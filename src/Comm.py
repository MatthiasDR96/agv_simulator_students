def tcp_write(destination, data):
    destination.put(data)


def sql_write(destination, data):
    destination.put(data)


def sql_read(request):
    return request.items


def sql_remove(request, order_number):
    request.get(lambda task: task.order_number == order_number)
