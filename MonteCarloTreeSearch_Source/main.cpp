#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <chrono>
#include <thread>
#include <boost/variant.hpp>

namespace py = pybind11;
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 rndm_nmbr_gnrt(seed);

class Node{
protected:
  Node* parent_;
  float success_;
  int nmbr_simls_;

  virtual bool IamLeaf() = 0;
public:
  Node(Node* parent): parent_{parent}, success_{0}, nmbr_simls_{0} {};

  float fitness(int parent_smls, float exploration_factor){
      float exploit = success_ / nmbr_simls_;
      float explore = exploration_factor * sqrt(log(parent_smls) / this->nmbr_simls_);
      return exploit + explore;
  }

  virtual Node* select( int &depth, float exploration_factor ) = 0;
  virtual void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node) = 0;
  virtual void expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState) = 0;
};

class WorldTurn: public Node{
private:
  struct WorldAction{
    std::vector < std::vector < int > > demand; // [store->[proudct -> demand]]
  };


  std::list<std::pair<WorldAction, Node*> > children;

  bool IamLeaf() override{
    return children.empty();
  }
  void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node) override {
    
    for(const auto child : children){
      if(child.second == node){
        for(const auto store: child.first.demand){
          std::transform(wareHouseState.begin(), wareHouseState.end(), store.begin(), wareHouseState.begin(), std::plus<int>{});
        }
      }
    }
    
    if(this->parent_){
      this->parent_->findWarehouseState(wareHouseState, this);
    }
  }
public:
  WorldTurn(Node* parent):Node{parent} {};
  Node* select(int &depth, float exploration_factor) override {
    if(this->IamLeaf())
      return this;
    int indexOfNodeToExplore = static_cast<int>(rndm_nmbr_gnrt() % children.size());
    /***
    Since a Monte Carlo Tree Simulation is used for advesarial games and reinforcement learning, 
    Not making this step random would imply the universe is creating demand in a way that minimises my
    success. Its not, its doing it probabalistically. The children are already creating probabalistically
    so exploring them will be random.  
    ***/
    std::list<std::pair<WorldAction, Node*> >::iterator data_ = children.begin();
    advance(data_, indexOfNodeToExplore);
    depth++;
    return data_->second->select(depth, exploration_factor); 
  }
  void expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState) override {

  }
};

class MyTurn : public Node{
private:
  struct MyAction{
    std::vector < std::pair< int, std::vector < int > > > trucks; //[truck->(store, [ product_id -> amount ])] 
  };
  
  
  std::list<std::pair<MyAction, Node*> > children;

  bool IamLeaf() override{
    return children.empty();
  }
  void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node) override {

    for(const auto child : children){
      if(child.second == node){
        for(const auto truck : child.first.trucks){
          std::transform(wareHouseState.begin(), wareHouseState.end(), truck.second.begin(), wareHouseState.begin(), std::minus<int>{});
        }
      }
    }

    if(this->parent_){
      this->parent_->findWarehouseState(wareHouseState, this);
    }
  }
public:
  MyTurn(Node* parent):Node{parent} {};
  Node* select(int &depth, float exploration_factor) override {
    if(this->IamLeaf())
      return this;
    depth++;
    Node* fitest_chld = nullptr;
    float crnt_bst_fitness = std::numeric_limits<float>::lowest();
    for(const auto data_ : children ){
      float crnt_fitness = data_.second->fitness(this->nmbr_simls_, exploration_factor);
      if( crnt_fitness > crnt_bst_fitness ){
        crnt_bst_fitness = crnt_fitness;
        fitest_chld = data_.second;
      }
    }
    return fitest_chld;
  }
};

class MonteCarloTreeSearchCpp {
private:
  Node* treeRoot;
  int widthOfFirstLevel_;
  int demand_min_;
  int demand_max_;
  int nmbr_strs_;
  int nmbr_prdcts_;
  int time_frm_;
  int DC_cpcty_;
  int nmbr_brnch_wrldTurn_;
  int nmbr_brnch_myTurn_;
  int nmbs_simulations_per_rollout_;
  int max_nmbr_trucks_;
  float exploration_factor_;
  std::vector<int32_t> wareHouseState_;
  std::vector<float> demand_prb_dstrbutn_;

public:
  MonteCarloTreeSearchCpp(int widthOfFirstLevel,int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,
  int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn,int nmbs_simulations_per_rollout, int max_nmbr_trucks, float exploration_factor,
   py::array_t<int32_t> wareHouseStatePy, py::array_t<float> demand_prb_dstrbutnPy) : 
    widthOfFirstLevel_{widthOfFirstLevel},
    demand_min_{demand_min},
    demand_max_{demand_max},
    nmbr_strs_{nmbr_strs},
    nmbr_prdcts_{nmbr_prdcts},
    time_frm_{time_frm},
    DC_cpcty_{DC_cpcty},
    nmbr_brnch_wrldTurn_{nmbr_brnch_wrldTurn},
    nmbr_brnch_myTurn_{nmbr_brnch_myTurn},
    nmbs_simulations_per_rollout_{nmbs_simulations_per_rollout},
    max_nmbr_trucks_{max_nmbr_trucks},
    exploration_factor_{exploration_factor} {
      py::buffer_info wareHouseState = wareHouseStatePy.request(); 
      py::buffer_info demand_prb_dstrbutn = demand_prb_dstrbutnPy.request(); 
      if(wareHouseState.ndim != 1)
        throw std::runtime_error("The states of warehouse should be a 1-D array");
      if(wareHouseState.shape[0] != nmbr_prdcts_)
        throw std::runtime_error("The wareHouseState length should be equal to the number of products");
      wareHouseState_.resize(nmbr_prdcts_);
      copy(static_cast<int32_t *>(wareHouseState.ptr), static_cast<int32_t *>(wareHouseState.ptr) + nmbr_prdcts, wareHouseState_.begin());

      int sm_of_prds = accumulate(wareHouseState_.begin(), wareHouseState_.end(), 0);
      if(sm_of_prds > DC_cpcty_)
        throw std::runtime_error("There are more products in the warehouse than the capacity, which is impossible.");

      if(demand_prb_dstrbutn.ndim != 4)
        throw std::runtime_error("Incorrect dimentions for the probability distribution");
      int demand_range = demand_max_ - demand_min_ + 1;
      if(demand_prb_dstrbutn.shape != std::vector<py::ssize_t>({nmbr_strs_, nmbr_prdcts_, time_frm_, demand_range}))
        throw std::runtime_error("Shape of probability distribution is not correct");

      int prb_dstrbutn_sz = nmbr_strs_ * nmbr_prdcts_ * time_frm_ * demand_range;
      demand_prb_dstrbutn_.resize(prb_dstrbutn_sz);
      copy(static_cast<float* >(demand_prb_dstrbutn.ptr), static_cast<float* >(demand_prb_dstrbutn.ptr) + prb_dstrbutn_sz, demand_prb_dstrbutn_.begin());

      treeRoot = new WorldTurn(nullptr);
    };


  
  void operator () (int number_of_iterations){
    while(number_of_iterations--){
      int depth = 0;
      Node* node = treeRoot->select(depth, this->exploration_factor_);
      {
        std::vector<int32_t> wareHouseStateCpy(wareHouseState_);
        node->expand( this->nmbr_brnch_wrldTurn_, this->nmbr_brnch_myTurn_, wareHouseStateCpy );
      }
    }
  }
};



PYBIND11_MODULE(PyMCTS, module_handle) {
  module_handle.doc() = "I'm a docstring hehe";
  py::class_<MonteCarloTreeSearchCpp>(
			module_handle, "MonteCarloTreeSearch"
			).def(py::init<int,int,int,int,int,int,int,int,int,int,int,float,py::array_t<int32_t> , py::array_t<float> >())
    ;
}