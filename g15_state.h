#ifndef GUARD_g15_state_h
#define GUARD_g15_state_h

#include <vector>
#include <string>
#include <list>

static const int gWidth = 4;

class State {
    static const int m_w = gWidth;
    static const int m_h = gWidth;
    std::vector < std::vector<int> > m_matrix;
    std::string m_key;
    State(const State& state) {this->copy(state);}
    bool find_hole(int &h_i, int &h_j) const;
    public:
    State();
    ~State(){}
    bool equal(const State& state) const;
    void copy(const State& state);
    void setup(const std::string *numbers = NULL);
    void print() const;
    const std::string& key() const {return m_key;}
    double heuristic(const State& goal) const;
    bool next_steps(std::list<State*>& steps) const;
    void update_key();
};

#endif /*GUARD_g15_state_h*/
