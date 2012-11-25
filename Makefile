OBJS = bcc_tool.o bcc_lib.o
CFLAGS = -Wall

all: $(OBJS)
	$(CC) $^ -o bcc_tool -lm

clean:
	rm -f *.o *~ core bcc_tool
