import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.StringTokenizer;

public class SteinerTree {
	static int N;									//Number of Nodes
	static int E;									//Number of Edges
	static int T;									//Number of Terminals
	
	static int[][] costMatrix;						// 2D Array to store costs (weights) of edges 	
	static LinkedList<Node>[] adjacencyList;		
	static Node[] node;									
	
	static void initialize() {
		costMatrix=new int[N][N];
		adjacencyList=new LinkedList[N];
		node=new Node[N];
		for(int i=0;i<N;i++) {
			node[i]=new Node(i);
			adjacencyList[i]=new LinkedList<Node>();			
		}
	}
	
	public static void main(String[] args) throws IOException {
		Reader.init(System.in);
		
		Reader.next();						//SECTION
		Reader.next();						//Graph
		Reader.next();						//Nodes
		N=Reader.nextInt();
		Reader.next();						//Edges
		E=Reader.nextInt();					
		
		initialize();
		
		for(int i=0;i<E;i++) {
			Reader.next();
			int a=Reader.nextInt()-1;
			int b=Reader.nextInt()-1;
			int c=Reader.nextInt();
			
			addEdge(node[a],node[b]);
			costMatrix[a][b]=c;
			costMatrix[b][a]=c;
		}
		
		Reader.next();						//END
		Reader.next();						//SECTION
		Reader.next();						//Terminals
		Reader.next();						//Terminals
		T=Reader.nextInt();
		
		for(int i=0;i<T;i++) {
			Reader.next();
			int t=Reader.nextInt()-1;
			
			node[t].isTerminal=true;
		}
		
		Reader.next();						//END
		
		//Start Here
		int[][] Closure_costMatrix=new int[N][N];					//2D Array for cost of paths which are taken as edges				
		LinkedList<Node>[] Closure_adjacencyList=new LinkedList[N];
		
		for(int i=0;i<N;i++) {
			Closure_adjacencyList[i]=new LinkedList<Node>();
		}
		
		for(int i=0;i<N;i++) {
			if(node[i].isTerminal==true) {
				Shortest_Paths.dijkstra(i, node, adjacencyList, costMatrix, N);
				
				for(int j=0;j<N;j++) {
					if(i!=j && node[j].isTerminal==true) {
						addEdge(Closure_adjacencyList, node[i], node[j]);		// adding fake edges
						Closure_costMatrix[i][j]=node[j].distance;				// adding fake costs
					}
					
					node[j].reset();
				}			
			}	//End if
		}	// End for
		
		long closure_cost=MST.prims(node, Closure_adjacencyList, Closure_costMatrix, N);
		
		NN_Pair PathstoConsider[]=new NN_Pair[T-1];								
		for(int i=0, t=0;i<N;i++) {
			if(node[i].isTerminal==true && node[i].prev!=null) {
				PathstoConsider[t]=new NN_Pair(i, node[i].prev.num);
				t++;
			}
		}
		
		for(int j=0;j<N;j++) {
			node[j].reset();
		}
		
		int adjacencyMatrix[][]=new int[N][N];
		for(int i=0;i<T-1;i++) {
			Shortest_Paths.dijkstra(PathstoConsider[i].n1, node, adjacencyList, costMatrix, N);
			Shortest_Paths.addPaths(node, adjacencyMatrix, PathstoConsider[i].n1, PathstoConsider[i].n2);
			
			for(int j=0;j<N;j++) {
				node[j].reset();
			}
		}
		
		long final_cost=0;
		int final_nodes=0;
		int final_edges=0;
		int [] final_node=new int[N];
		for(int i=0;i<N;i++) {
			for(int j=0;j<N;j++) {
				if(adjacencyMatrix[i][j]==1) {
					final_cost=final_cost+costMatrix[i][j];
					final_node[i]=1;
					final_node[j]=1;
					final_edges++;
				}
			}
		}
		for(int i=0;i<N;i++) {
			if(final_node[i]==1) {
				final_nodes++;
			}
		}
		
		//Output
		System.out.println("SECTION Graph");
		System.out.println("Nodes "+final_nodes);
		System.out.println("Edges "+final_edges);
		System.out.println();
		
		for(int i=0;i<N;i++) {
			for(int j=0;j<N;j++) {
				if(adjacencyMatrix[i][j]==1) {
					System.out.println("E "+(i+1)+" "+(j+1)+" "+costMatrix[i][j]);
				}
			}
		}
		
		System.out.println("END");
		System.out.println("SECTION Terminals");
		for(int i=0;i<N;i++) {
			if(node[i].isTerminal==true) {
				System.out.println("T "+(node[i].num+1));
			}
		}
		System.out.println("END");
		System.out.println("SECTION Cost");
		System.out.println(final_cost);
		System.out.println("END");
	}
	
	static void addEdge(Node source, Node dest){
		adjacencyList[source.num].add(dest);
		adjacencyList[dest.num].add(source);
	}
	
	static void addEdge(LinkedList<Node>[] L, Node source, Node dest){
		L[source.num].add(dest);
		L[dest.num].add(source);
	}
}

class Shortest_Paths{
	
	static void dijkstra(int source, Node[] V, LinkedList<Node>[] L, int[][] costMatrix, int N){
		V[source].distance=0;
		PriorityQueue<ND_Pair> PQ=new PriorityQueue<ND_Pair>(new myComparator());
		PQ.add(new ND_Pair(V[source].num, V[source].distance));
		while(PQ.isEmpty()==false){
			ND_Pair v=PQ.poll();
			Node vertex=V[v.n];
			
			if(vertex.visited==true) {								// To bypass Duplicate vertices
				continue;
			}
			vertex.visited=true;
			int size=L[vertex.num].size();
			
			for(int j=0;j<size;j++){
				Node neighbour=L[vertex.num].get(j);
				int temp=vertex.distance+costMatrix[vertex.num][neighbour.num];
				
				if(neighbour.visited ==false && temp<neighbour.distance){
					neighbour.distance=temp;
					neighbour.prev=vertex;
					PQ.add(new ND_Pair(neighbour.num, neighbour.distance));
				}
			}
		}	//End while
	}

	static void addPaths(Node[] V, int[][] adjacencyMatrix, int source, int dest) {		
		Node current=V[dest];
		Node previous=V[dest].prev;
		while(previous!=null) {
			int min=Math.min(current.num, previous.num);
			int max=Math.max(current.num, previous.num);
			adjacencyMatrix[min][max]=1;
			current=previous;
			previous=previous.prev;
		}
	}
}

class MST{
	
	static long prims(Node[] V, LinkedList<Node>[] L, int[][] cost, int N){	
		int x;
		for(x=0;x<N;x++) {
			if(V[x].isTerminal==true) {						// Finding the first terminal
				V[x].distance=0;
				break;
			}
		}

		PriorityQueue<ND_Pair> PQ=new PriorityQueue<ND_Pair>(new myComparator());
		PQ.add(new ND_Pair(V[x].num, V[x].distance));
		
		while(PQ.isEmpty()==false) {
			ND_Pair vd=PQ.poll();
			Node v=V[vd.n];
			if(v.visited==true) {
				continue;
			}
			v.visited=true;
			int n=L[v.num].size();							// Number of neighbours
			for(int i=0;i<n;i++){
				Node neighbour=L[v.num].get(i);
				if(neighbour.visited==false && cost[v.num][neighbour.num]<neighbour.distance) {
					neighbour.prev=v;
					neighbour.distance=cost[v.num][neighbour.num];					// Updating Cost of roads to neighbour city
					PQ.add(new ND_Pair(neighbour.num, neighbour.distance));
				}
			}
			
		}	//End while
	
		long sum=0;
		for(int i=0;i<N;i++) {
			if(V[i].isTerminal==true) {
				sum=sum+V[i].distance;
			}
		}

		return sum;
	}
}

class myComparator implements Comparator<ND_Pair>{				//Comparator for Priority Queue
	public int compare(ND_Pair v1, ND_Pair v2) {
		return v1.d-v2.d;
	}
}

class Node{
	int num;
	int distance;
	boolean isTerminal;
	
	boolean visited;
	Node prev;
	
	public Node(int i){
		num=i;
		isTerminal=false;
		
		distance=Integer.MAX_VALUE;
		visited=false;
		prev=null;
		
	}
	
	public void reset() {
		distance=Integer.MAX_VALUE;
		visited=false;
		prev=null;
	}
}

class ND_Pair{										// Class for node-distance pair
	int n;
	int d;
	public ND_Pair(int n, int d){
		this.n=n;
		this.d=d;
	}
}

class NN_Pair{										// Class for node-node pair
	int n1;
	int n2;
	public NN_Pair(int n1, int n2) {
		this.n1=n1;
		this.n2=n2;
	}
}

/** Class for buffered reading int and double values */
class Reader {
    static BufferedReader reader;
    static StringTokenizer tokenizer;

    /** call this method to initialize reader for InputStream */
    static void init(InputStream input) {
        reader = new BufferedReader(
                     new InputStreamReader(input) );
        tokenizer = new StringTokenizer("");
    }

    /** get next word */
    static String next() throws IOException {
        while ( ! tokenizer.hasMoreTokens() ) {
            //TODO add check for eof if necessary
            tokenizer = new StringTokenizer(
                   reader.readLine() );
        }
        return tokenizer.nextToken();
    }

    static int nextInt() throws IOException {
        return Integer.parseInt( next() );
    }
	
    static double nextDouble() throws IOException {
        return Double.parseDouble( next() );
    }
}