'''
Adaptado de:
@author: Edielson - https://github.com/edielsonpf
'''

from SearchAlgorithms.greedy_search import greedy_search
import networkx as nx

import matplotlib
matplotlib.use('TkAgg')

try:
    import matplotlib.pyplot as plt
except:
    raise

def printPath(path,start):
    string=(start)
    for city in path:
        if city != start:
            string=(string+' -> '+city)
    print(string)
    
def plotGraph(G,option,position=None):
    """Plot a graph G with specific position.

    Parameters
    ----------
    G : NetworkX graph
    option : if 1, edges with weight greater then 0 are enlarged. The opposite happens for option equal to 0.
    position : nodes position 
    
    Returns
    -------
    position: nodes position generated during plot (or same positions if supplied).

    """
    if option == 1:
        elarge=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] > 0]
        esmall=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] <= 0]
    else:
        elarge=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] <= 0]
        esmall=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] > 0]
        
    
    if position == None:
        position=nx.spring_layout(G) # positions for all nodes
    
    # nodes
    nx.draw_networkx_nodes(G,position,node_size=500)
        
    # edges
    nx.draw_networkx_edges(G,position,edgelist=elarge,width=2)
    nx.draw_networkx_edges(G,position,edgelist=esmall,width=2,alpha=0.5,edge_color='b',style='dashed')
    
    # labels
    nx.draw_networkx_labels(G,position,font_size=20,font_family='sans-serif')
    
    plt.axis('off')
    #plt.savefig("weighted_graph.png") # save as png
    plt.show() # display
    
    return position


class FindPath(object): 
    '''
    classdocs
    '''

    def __init__(self, graph):
        '''
        Constructor
        '''
        self.problem = graph
        
    def ObjectiveTest(self,current,target): 
        """Return ``True`` if ``current`` state corresponds to the ``target`` state 
        """ 
        solution = False 
        if current == target:
            solution = True
        return  solution
 
    def ExpandSolution(self,current): 
        """Returns all possible states from ``current`` 
        """ 
        return  self.problem.neighbors(current)
    
    def Heuristic(self,target,current): 
        """Returns heuristic associated to ``current`` 
        """ 
        custo_linha_reta={('Pouso Alegre','Belo Horizonte'):350,
                      ('Santa Rita','Belo Horizonte'):380,
                      ('Itajuba','Belo Horizonte'): 500,
                      ('Cachoeira de Minas','Belo Horizonte'):400,
                      ('Varginha','Belo Horizonte'):250,
                      ('Congonhal','Belo Horizonte'):420,
                      ('Ouro Fino','Belo Horizonte'):530,
                      ('Belo Horizonte','Belo Horizonte'):0}
        
        Heuristic = custo_linha_reta.get((current, target))
        print (Heuristic)                
        return Heuristic
    
          
    
    
if __name__ == '__main__':
    
    nodes = ['Pouso Alegre','Santa Rita','Varginha','Congonhal',
             'Cachoeira de Minas','Itajuba','Belo Horizonte','Ouro Fino']

    edges=[('Pouso Alegre','Santa Rita'),('Pouso Alegre','Varginha'),('Pouso Alegre','Congonhal'),('Pouso Alegre','Cachoeira de Minas'),
           ('Santa Rita','Pouso Alegre'),('Santa Rita','Itajuba'),('Santa Rita','Cachoeira de Minas'),
           ('Itajuba','Santa Rita'),
           ('Varginha','Belo Horizonte'),('Varginha','Pouso Alegre'),
           ('Cachoeira de Minas','Santa Rita'),('Cachoeira de Minas','Pouso Alegre'),
           ('Congonhal','Pouso Alegre'),('Congonhal','Ouro Fino'),
           ('Belo Horizonte','Varginha'),('Ouro Fino','Congonhal')]
    
    
    cost={ ('Pouso Alegre','Santa Rita'):30,
           ('Pouso Alegre','Varginha'):120,
           ('Pouso Alegre','Congonhal'):35,
           ('Pouso Alegre','Cachoeira de Minas'):40,
           ('Santa Rita','Pouso Alegre'): 30,
           ('Santa Rita','Itajuba'):50,
           ('Santa Rita','Cachoeira de Minas'):45,
           ('Itajuba','Santa Rita'):50,
           ('Varginha','Belo Horizonte'):250,
           ('Varginha','Pouso Alegre'):120,
           ('Cachoeira de Minas','Santa Rita'):45,
           ('Cachoeira de Minas','Pouso Alegre'):40,
           ('Congonhal','Pouso Alegre'):35,
           ('Congonhal','Ouro Fino'):22,
           ('Belo Horizonte','Varginha'):250,
           ('Ouro Fino','Congonhal'):22 }
    
    
    
    G=nx.DiGraph()
    
    G.add_nodes_from(nodes)
    
    #Adding the respective cost for each edge in the graph
    for u,v in edges:
        G.add_edge(u, v, weight=cost[u,v])
    positions = plotGraph(G, 1, None)
            
    #Creating an problem object based on FindPath class
    Problema = FindPath(G)
    
    #Creating an object for breadth first search algorithm for ``FindPath`` problem
    SearchObj = greedy_search(Problema)    
    
    
    start = 'Pouso Alegre'
    target = 'Belo Horizonte'
    print('\nSearching %s starting from %s...'%(target,start))
    solution,path,path_edges = SearchObj.search(start,target)
    print('Done!\n')
    if solution:
        print('Path found!')
        printPath(path,start)
        for u,v in edges:
            if (u,v) not in path_edges:
                G.remove_edge(u, v)
        plotGraph(G, 1, positions)        
    else:
        print('Path not found!')        