#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/variant.hpp>
namespace py = pybind11;

struct WorldAction{
  std::vector < std::vector < int > > demand; // [store->[proudct -> demand]]
};
struct MyAction{
  std::vector < std::vector < std::pair<int,int> > > trucks; //truck->[ product_id, amount ] 
  std::vector < int > factory_produce; //[product_id -> amount produced] 
};
class Node{
protected:
  Node* parent_;
  float success_;
  int nmbr_simls_;
  std::list < std::pair < boost::variant < WorldAction, MyAction >, Node* > > children;



  bool IamLeaf(){
    return children.empty();
  }
public:
  Node(Node* parent): parent_{parent}, success_{0}, nmbr_simls_{0} {};

  // virtual Node* select(int sibling_cnt) = 0;
};

class WorldTurn: public Node{
private:

public:
  WorldTurn(Node* parent):Node{parent} {};


};

class MyTurn : public Node{
private:

public:
  MyTurn(Node* parent):Node{parent} {};

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
        // Node* node = treeRoot->select(depth);
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