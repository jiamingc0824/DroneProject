import threading

x = [0, 1, 2, 3, 4, 5, 6]
threads = []


def PrintArray(arr):
    for i in arr:
        print i


def ModifyArray(arr):
    arr[0] = 99


PRINT = threading.Thread(target=PrintArray, args=(x,))
MODIFY = threading.Thread(target=ModifyArray, args=(x,))
threads.append(PRINT)
threads.append(MODIFY)
MODIFY.start()
PRINT.start()


for t in threads:
    t.join()

for i in x:
    print i
