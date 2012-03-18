/*
Fast and dirty realization of A* search algorithm for game '15'
$Id: game15-astar.cpp,v 1.4 2011/10/16 18:36:52 oraz Exp $
*/
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cassert>
#include <unistd.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <map>
#include <list>
#include <set>

#include "g15_state.h"

static bool manhattan = true;
static double gGreediness = 1.05;

class Node {
    State m_state;
    std::string m_action;
    double m_cost;
    double m_gval;
    Node *m_parent;
    Node(const Node&);
    public:
    Node(const State& state, Node* parent, const std::string action,
        double cost, const State& goal);
    ~Node();
    inline void print(void) const {m_state.print();}
    inline const std::string& key() const {return m_state.key();}
    inline bool is_goal(const State& goal) const {return m_state.equal(goal);}

    inline double g() const {return m_gval;}
    inline bool next_steps(std::list<State*>& steps) const
        {return m_state.next_steps(steps);}
    inline Node* parent() const {return m_parent;}

};

Node::Node(const State& state, Node* parent, const std::string action,
    double cost, const State& goal)
{
    m_state.copy(state);
    m_action = action;
    m_parent = parent;
    
    if(m_parent)
        m_cost = m_parent->m_cost;
    else
        m_cost = 0;
    m_cost += cost;

    // rise greediness with grow of the path
    m_cost /= gGreediness;
    
    m_gval = m_cost + m_state.heuristic(goal);
}

struct NodePtrLess {
   bool operator( )(const Node* p1,
                   const Node* p2) {
   assert(p1 && p2);
   return(p1->key() < p2->key());
   }
};
struct NodePtrGLess {
   bool operator( )(const Node* p1,
                   const Node* p2) {
   assert(p1 && p2);
     if(p1->g() == p2->g())
         return (p1->key() < p2->key()); 
   return (p1->g() < p2->g());
   }
};

typedef std::map<std::string, Node*> mapNode;
typedef std::set<Node*, NodePtrGLess> multisetNode;

class Search {
    State m_start;
    State m_goal;
    Node* m_head;
    Search(const Search&);
    mapNode m_frontier;
    multisetNode m_queue;
    mapNode m_expanded;
    public:
    Search(const State& start, const State& goal) ;
    ~Search();
    void print_frontier();
    Node* run();
};

Search::Search(const State& start, const State& goal)
{
    m_start.copy(start);
    m_goal.copy(goal);
    m_head = new Node(start, NULL, "begin", 0, m_goal);
    //m_head->print();
    m_frontier[m_head->key()] = m_head;
    m_queue.insert(m_head);
}

Node* Search::run()
{
    int states_expanded = 0;

    while(!m_frontier.empty())
    {
        Node *best = NULL; 
#if 0
        for(mapNode::iterator p = m_frontier.begin(); p != m_frontier.end(); ++p)
        {
            double g = p->second->g();
            if(best) {
                if(g < best->g()) {
                    best = p->second;
                }
            } else {
                best = p->second;
            }
        }
        assert(best);
        assert(!m_queue.empty());
        Node * bestQueue = *(m_queue.begin());
        if(best->g() != bestQueue->g()) {
            std::cout << "not best: " << best->g() << " != " << bestQueue->g() << std::endl;
            for(multisetNode::const_iterator it = m_queue.begin(); it != m_queue.end(); ++it)
            {
                using namespace std;
                Node* node = *it;
                cout << "\t" << node->key() << " => g=" << node->g() << endl;
            }
        }
#else
        best = *(m_queue.begin());
#endif
        
        if(best->is_goal(m_goal)) return best;
        
        // expand the best
        size_t size = m_frontier.size();
        m_frontier.erase(best->key());
        if(m_frontier.size() != size - 1 ) 
            {
                std::cerr<<"oops: " << m_frontier.size() << " != " << size - 1 << std::endl;
            }
        m_queue.erase(best);
        m_expanded[best->key()] = best; 
        //best->print();
        states_expanded++;
        if(states_expanded%1000 == 0) {
            std::cerr << "moves: " << states_expanded
            << " frontier: " << m_frontier.size()
            << " expanded: " << m_expanded.size()
            << std::endl;
        }

        std::list<State*> steps;
        if(!best->next_steps(steps)) {
                throw std::runtime_error("no moves!");
        }
        //std::cout<< "steps=" << steps.size() << std::endl;
        for(std::list<State*>::iterator it = steps.begin(); it != steps.end(); ++it)
        {
            State *s = *it;
            /* THIS is AN ERROR: we can add same state into frontier
            if(!m_frontier.empty() && m_frontier.find(s->key()) != m_frontier.end()) {
                //std::cout << "frontier old step:" << std::endl; (*it)->print();
                continue;
            }
            */
            if(!m_expanded.empty() && m_expanded.find(s->key()) != m_expanded.end()) { 
                //std::cout << "expanded old step:" << std::endl; (*it)->print();
                continue;
            }
            Node *node = new Node(*s, best, "?", 1, m_goal);
            m_frontier[node->key()] = node;
            m_queue.insert(node);
            //std::cout << "step:" << std::endl; node->print();
        }
        //int hit; std::cin >> hit;
    }
    return NULL;
}


Search::~Search()
{
    //delete m_head and all nodes;
}

void Search::print_frontier()
{    
    const mapNode& m = m_frontier;
    for(mapNode::const_iterator p = m.begin(); p != m.end(); ++p)
    {
        p->second->print();
        std::cout << std::endl;
    }
}

#if 1
// best path = 60 moves
// more then 13 mln expanded states with gridiness=1
// 3000 expanded states with gridiness=1.01
static    const std::string start_str("\
13 12 11  6 \
 3  5  1  9 \
 4  2 14 10 \
 8  7 15  0 \
");
#else
static    const std::string start_str("\
 1  2  3  4 \
 5  6  7  8 \
 9 10 11 12 \
15 13 14  0 \
");
#endif

void usage(const std::string& cmdline)
{
    std::cerr << "Usage: " << cmdline << " -m -g<greediness>" << std::endl;
    exit(1);
}

int main(int argc, char* argv[])
{
    int mflag, ch;

    mflag = 0;
    while ((ch = getopt(argc, argv, "mg:")) != -1) {
        switch (ch) {
            case 'm':
                mflag = 1;
                break;
            case 'g':
                gGreediness = atof(optarg);
                std::cout << "using greediness=" << gGreediness << std::endl;
                break;
            case '?':
            default:
                usage(argv[0]);
        }
    }
    argc -= optind;
    argv += optind;

    State startState, goalState;
    goalState.setup();

    srandom(time(NULL));
    
    if(argc > 0) {
        int n = atoi(argv[0]);
        startState.setup();
        // make n random steps back from the goal position
        for(int i = 0; i < n; ++i)
        {
            std::list<State*> steps;
            if(!startState.next_steps(steps)) {
                throw std::runtime_error("no moves!");
            }
            int k = random() % steps.size();
             int j = 0;
            for(
                std::list<State*>::iterator it = steps.begin(); 
                it != steps.end(); 
                ++it
                )
            {
                State *s = *it;
                if(j == k)
                    startState.copy(*s);
                delete s;
                ++j;
            }
        }
    } else {
        startState.setup(&start_str);
    }

    Search search(startState, goalState);
    std::cout << "search..";
    Node *end = search.run();
    if(!end) {
        std::cerr << "Path not found!" << std::endl;
        return 0;
    }
    std::cout << "ok" << std::endl;
    std::list<Node*> path;
    Node *cur = end;
    while(cur) {
        path.push_front(cur);
        cur=cur->parent();
    }
    if(path.size() < 10) {
        std::cout << "A* search result:" << std::endl;
        for(std::list<Node*>::iterator it = path.begin(); it != path.end(); ++it)
        {
            (*it)->print();
        }
    }
    std::cout << "A* search result: " << path.size() - 1 << " moves in the path." << std::endl;
    std::cout << "start position:" << std::endl;
    startState.print();

    return 0;
}
