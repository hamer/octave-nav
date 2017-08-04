PROG = fusion
OBJDIR = .

CFLAGS += -std=gnu99 -W -Wall -pedantic -O0 -fstack-check
LDFLAGS += -lm
SOURCES = main.c vector.c matrix.c quaternion.c rotation.c geod.c
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SOURCES))

.PHONY: all clean
all: $(OBJDIR)/$(PROG)

clean:
	-rm -f $(OBJS) $(OBJDIR)/$(PROG)

$(OBJDIR)/$(PROG): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
