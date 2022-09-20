#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <chrono>
#include <thread>
#include <iostream>
namespace py = pybind11;

class Node{
protected:
  Node* parent_;
  float success_;
  int nmbr_simls_;
public:
  Node(Node* parent): parent_{parent}, success_{0}, nmbr_simls_{0} {};
};

class WorldTurn: private Node{
private:

};

class MyTurn : private Node{
private:

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
  std::vector<int32_t> wareHouseState_;
  std::vector<float> demand_prb_dstrbutn_;

public:
  MonteCarloTreeSearchCpp(int widthOfFirstLevel,int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,int nmbr_brnch_wrldTurn, 
    int nmbr_brnch_myTurn, py::array_t<int32_t> wareHouseStatePy, py::array_t<float> demand_prb_dstrbutnPy) : 
    widthOfFirstLevel_{widthOfFirstLevel},
    demand_min_{demand_min},
    demand_max_{demand_max},
    nmbr_strs_{nmbr_strs},
    nmbr_prdcts_{nmbr_prdcts},
    time_frm_{time_frm},
    DC_cpcty_{DC_cpcty},
    nmbr_brnch_wrldTurn_{nmbr_brnch_wrldTurn},
    nmbr_brnch_myTurn_{nmbr_brnch_myTurn}{
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
      if(demand_prb_dstrbutn.shape != std::vector<py::ssize_t>({nmbr_strs_, nmbr_prdcts_, time_frm_, demand_range}));
        throw std::runtime_error("Shape of probability distribution is not correct");

      int prb_dstrbutn_sz = nmbr_strs_ * nmbr_prdcts_ * time_frm_ * demand_range;
      demand_prb_dstrbutn_.resize(prb_dstrbutn_sz);
      copy(static_cast<float* >(demand_prb_dstrbutn.ptr), static_cast<float* >(demand_prb_dstrbutn.ptr) + prb_dstrbutn_sz, demand_prb_dstrbutn_.begin());
    };
};


PYBIND11_MODULE(PyMCTS, module_handle) {
  module_handle.doc() = "I'm a docstring hehe";
  py::class_<MonteCarloTreeSearchCpp>(
			module_handle, "MonteCarloTreeSearch"
			).def(py::init<int,int,int,int,int,int,int,int,int,py::array_t<int32_t> , py::array_t<float> >())
    ;
}