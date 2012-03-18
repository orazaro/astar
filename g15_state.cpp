#include "g15_state.h"

State::State()
{
    m_matrix = std::vector < std::vector<int> >(m_w , std::vector<int>(m_h,0));
    this->update_key();
}

void State::copy(const State& state)
{
    for(int i = 0; i < m_w; ++i)
    {
        for(int j = 0; j < m_h; ++j)
        {
            m_matrix[i][j] = state.m_matrix[i][j];
        }
    }
    this->update_key();
}

bool State::equal(const State& state) const
{
    for(int i = 0; i < m_w; ++i)
    {
        for(int j = 0; j < m_h; ++j)
        {
            if(m_matrix[i][j] != state.m_matrix[i][j])
                return false;
        }
    }
    return true;
}

void State::print() const
{
    for(int i = 0; i < m_w; ++i)
    {
        for(int j = 0; j < m_h; ++j)
        {
            std::cout << std::setw(2) << m_matrix[i][j] << " ";
        }
        //if(0 == i)
            //std::cout << "\tkey: " << m_key;
        std::cout << std::endl;
    }
    std::cout << "=========================" << std::endl;
}

void State::setup(const std::string * numbers)
{
    if(numbers) {
        std::string word;
        std::istringstream iss(*numbers, std::istringstream::in);

        for(int i = 0; i < m_w; ++i)
        for(int j = 0; j < m_h; ++j)
        {
            if(iss >> word) {
                int num = atoi(word.c_str());
                m_matrix[i][j] = num;
            } else {
                throw std::runtime_error("bad numbers:[" + *numbers + "]");
            }
        }
    }
    else {
        int cur = 1;
        for(int i = 0; i < m_w; ++i)
        for(int j = 0; j < m_h; ++j)
        {
            m_matrix[i][j] = cur;
            cur++;
        }
        m_matrix[m_w-1][m_h-1] = 0;
    }
}

void State::update_key()
{
    char buf[BUFSIZ]; int len = 0;

    for(int i = 0; i < m_w; ++i)
    for(int j = 0; j < m_h; ++j)
    {
        len += snprintf(buf+len,BUFSIZ,"%d.", m_matrix[i][j]);
    }
    m_key = std::string(buf);
}

double State::heuristic(const State& goal) const
{
    double dist = 0;
    if(manhattan)
    {
        for(int i = 0; i < m_w; ++i)
        for(int j = 0; j < m_h; ++j)
        {
            if(!goal.m_matrix[i][j]) continue;
            for(int ii = 0; ii < m_w; ++ii)
            for(int jj = 0; jj < m_h; ++jj)
            {
                if(m_matrix[ii][jj] == goal.m_matrix[i][j])
                {
                    dist += fabs(i - ii) + fabs(j - jj);
                }
            }
        }
    }
    else
    {
        for(int i = 0; i < m_w; ++i)
        for(int j = 0; j < m_h; ++j)
        {
            if(m_matrix[i][j] != goal.m_matrix[i][j])
                dist += 1.;
        }
    }
        return dist;
}

bool State::find_hole(int &h_i, int &h_j) const
{
    bool found_hole = false;
    for(int i = 0; i < m_w && !found_hole; ++i)
    for(int j = 0; j < m_h && !found_hole; ++j)
    {
        if(m_matrix[i][j] == 0) {
            found_hole = true;
            h_i = i;
            h_j = j;
        }
    }
    return found_hole;
}

bool State::next_steps(std::list<State*>& steps) const
{
    int h_i = 0, h_j = 0;
    if(!find_hole(h_i,h_j)) return false;
    //left
    if( h_i > 0 ) {
        State* step = new State;
        step->copy(*this);
        step->m_matrix[h_i][h_j] = step->m_matrix[h_i-1][h_j];
        step->m_matrix[h_i-1][h_j] = 0;
        step->update_key();
        steps.push_back(step);
    }
    //right
    if( h_i < m_w - 1 ) {
        State* step = new State;
        step->copy(*this);
        step->m_matrix[h_i][h_j] = step->m_matrix[h_i+1][h_j];
        step->m_matrix[h_i+1][h_j] = 0;
        step->update_key();
        steps.push_back(step);
    }
    //up
    if( h_j > 0 ) {
        State* step = new State;
        step->copy(*this);
        step->m_matrix[h_i][h_j] = step->m_matrix[h_i][h_j-1];
        step->m_matrix[h_i][h_j-1] = 0;
        step->update_key();
        steps.push_back(step);
    }
    //down
    if( h_j < m_h - 1 ) {
        State* step = new State;
        step->copy(*this);
        step->m_matrix[h_i][h_j] = step->m_matrix[h_i][h_j+1];
        step->m_matrix[h_i][h_j+1] = 0;
        step->update_key();
        steps.push_back(step);
    }
    return true;
}

