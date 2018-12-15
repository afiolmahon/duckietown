import time
import ducky_bot
import ducky_graph
import ducky_io

TIMESTEP = 0.5

def main():
    graph = ducky_graph.DuckyGraph()
    bot_io = ducky_io.TestIO()
    start_node_id = 'A'
    start_orientation = 0
    bot = ducky_bot.DuckyBot(graph, bot_io, start_node_id, start_orientation)
    while True:
        bot.state_machine()
        time.sleep(TIMESTEP)   

if __name__ == '__main__':
    main()