#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <chrono>
#include <thread>
#include <boost/variant.hpp>

class MyTurn;

namespace py = pybind11;
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 rndm_nmbr_gnrt(seed);

int rowMajor4D(int n1,int n2, int n3, int n4, int N1, int N2, int N3, int N4){
  return n4 + N4 * ( n3 + N3 * ( n2 + N2 * n1 ) );
}

int binarySearchFirstLargerIndex(const std::vector<float> &data_, float value, int left, int right){
  /***
  Given a float vector and an index [l,r) it finds the first index i such that data_[i] >= value
  assuming it exists, if not it returns last element.
  ***/
  value = std::min(value, data_[ right - 1 ]); //in case total probability in 0.998 and i send 0.999 as value 
  while (left != right) {
    int mid = (left + right) / 2;
    if (data_[mid] < value) {
        left = mid + 1;
    }
    else {
        right = mid;
    }
  }
  return left;
}

class Node{
protected:
  Node* parent_;
  float success_;
  int nmbr_simls_;
  int depth_;
  virtual bool IamLeaf() = 0;
  Node(Node* parent, int depth): parent_{parent}, success_{0}, nmbr_simls_{0}, depth_{depth} {};
public:

  float fitness(int parent_smls, float exploration_factor);

  virtual Node* select( float exploration_factor ) = 0;
  virtual void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node) = 0;
  virtual void expand(  int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
    int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min) = 0;
};

class WorldTurn: public Node{
private:
  struct WorldAction{
    std::vector < std::vector < int > > demand; // [store->[proudct -> demand]]

    WorldAction(int nmbr_strs, int nmbr_prdcts) :demand{nmbr_strs, std::vector<int>(nmbr_prdcts)} {};
  };


  std::list<std::pair<WorldAction, Node*> > children;

  bool IamLeaf() override;
  void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node) override;
public:
  WorldTurn(Node* parent, int depth):Node{parent, depth} {};
  Node* select(float exploration_factor) override ;
  void expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
    int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min) override ;
};

class MyTurn : public Node{
private:
  struct MyAction{
    std::vector < std::pair< int, std::vector < int > > > trucks; //[truck->(store, [ product_id -> amount ])] 
    std::vector<int> total_prduc; //[product -> total amount(includes whats stocked and what sent in trucks)]
  };
  
  
  std::list<std::pair<MyAction, Node*> > children;

  bool IamLeaf() override ; 
  void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node) override ;

public:
  MyTurn(Node* parent, int depth):Node{parent, depth} {};
  Node* select(float exploration_factor) override ;
  void expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
    int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min) override ;
};

class MonteCarloTreeSearchCpp {
private:
  Node* treeRoot;
  int widthOfFirstLevel_;
  int demand_min_;
  int demand_max_;
  int demand_range_;
  int nmbr_strs_;
  int nmbr_prdcts_;
  int time_frm_;
  int DC_cpcty_;
  int nmbr_brnch_wrldTurn_;
  int nmbr_brnch_myTurn_;
  int nmbr_simulations_per_rollout_;
  int max_nmbr_trucks_;
  float exploration_factor_;
  std::vector<int32_t> wareHouseState_;
  std::vector<float> demand_prb_dstrbutn_;
  std::vector<float> cmltv_demand_prb_dstrbutn_;

public:
  MonteCarloTreeSearchCpp(int widthOfFirstLevel,int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,
  int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn,int nmbr_simulations_per_rollout, int max_nmbr_trucks, float exploration_factor,
   py::array_t<int32_t> wareHouseStatePy, py::array_t<float> demand_prb_dstrbutnPy) : 
    // widthOfFirstLevel_{widthOfFirstLevel},
    demand_min_{demand_min},
    demand_max_{demand_max},
    nmbr_strs_{nmbr_strs},
    nmbr_prdcts_{nmbr_prdcts},
    time_frm_{time_frm},
    DC_cpcty_{DC_cpcty},
    nmbr_brnch_wrldTurn_{nmbr_brnch_wrldTurn},
    nmbr_brnch_myTurn_{nmbr_brnch_myTurn},
    nmbr_simulations_per_rollout_{nmbr_simulations_per_rollout},
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
      int demand_range_ = demand_max_ - demand_min_ + 1;
      if(demand_prb_dstrbutn.shape != std::vector<py::ssize_t>({nmbr_strs_, nmbr_prdcts_, time_frm_, demand_range_}))
        throw std::runtime_error("Shape of probability distribution is not correct");

      int prb_dstrbutn_sz = nmbr_strs_ * nmbr_prdcts_ * time_frm_ * demand_range_;
      demand_prb_dstrbutn_.resize(prb_dstrbutn_sz);
      copy(static_cast<float* >(demand_prb_dstrbutn.ptr), static_cast<float* >(demand_prb_dstrbutn.ptr) + prb_dstrbutn_sz, demand_prb_dstrbutn_.begin());

      cmltv_demand_prb_dstrbutn_.resize(prb_dstrbutn_sz);
      for(int store = 0 ; store < nmbr_strs_ ; store++){
        for(int product = 0 ; product < nmbr_prdcts_ ; product++){
          for(int time = 0 ; time < time_frm_ ; time++){
            int strt_indx = rowMajor4D(store, product, time, 0, nmbr_strs_, nmbr_prdcts_, time_frm_, demand_range_);
            cmltv_demand_prb_dstrbutn_[strt_indx] = demand_prb_dstrbutn_[strt_indx]; 
            for(int demand = 1 ; demand < demand_range_ ; demand++){
              cmltv_demand_prb_dstrbutn_[strt_indx + demand] = cmltv_demand_prb_dstrbutn_[strt_indx + demand - 1] + demand_prb_dstrbutn_[strt_indx + demand];
            }
          }
        }
      }

      if(widthOfFirstLevel == -1){
        widthOfFirstLevel_ = demand_range_;
      }else if (widthOfFirstLevel <= demand_range_ && widthOfFirstLevel > 0){
        widthOfFirstLevel_ = widthOfFirstLevel;
      }else{
        throw std::runtime_error("Width of first level not sent properly. It should be -1 for max, or in [1, max]");
      }

      treeRoot = new WorldTurn(nullptr, 0);
    };


  
  void operator () (int number_of_iterations);
};

