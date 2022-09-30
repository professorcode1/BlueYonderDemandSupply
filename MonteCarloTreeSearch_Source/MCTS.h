#include <bits/stdc++.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <chrono>
#include <thread>

class MyTurn;

namespace py = pybind11;
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 rndm_nmbr_gnrt(seed);
float get_rndm_nmbr_nrml(){
  //return random float b/w [0,1]
  return static_cast<float>(static_cast<int64_t>(rndm_nmbr_gnrt()) - static_cast<int64_t>(std::mt19937::min())) / static_cast<float>(static_cast<int64_t>(std::mt19937::max()) - static_cast<int64_t>(std::mt19937::min()));
}
int get_rndm_int(int l, int r){
  //return random int between [l, r)
  int diff = r - l;
  int rndm_nmbr = abs(static_cast<int>(rndm_nmbr_gnrt()));
  return l + (rndm_nmbr % diff);
}
int get_rndm_elmnt_set(const std::set<int> &s){
  int index = get_rndm_int(0, s.size());
  std::set<int>::const_iterator data = s.begin();
  std::advance(data, index);
  return *(data);
}
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
void combinationUtil(std::vector<int32_t> &data, int start, int end, int index, int r) {
	
  if (index != r){
    int i_lower = start;
    int i_upper = std::min(end,end + 1 - r + index);

    int i = get_rndm_int( i_lower, i_upper + 1 );
    data[index] = i;
    combinationUtil(data, i+1, end, index+1, r);
  }

}
std::vector<int32_t> BeggarsAlgorithm(int n, int r){
	//one random solution to x1+x2+x3...xn = r
  int total = n + r - 1, choose = n - 1;
  std::vector<int32_t> data(n);
  combinationUtil(data, 0, total - 1, 0, choose );
  data[n-1] = total - data[n-2] - 1;
  for(int i = n-2 ; i> 0 ; i--)
    data[i] = data[i] - data[i-1] - 1;
  return data;
}
struct WorldAction{
  std::vector < std::vector < int > > demand; // [store->[proudct -> demand]]
  WorldAction(int nmbr_strs, int nmbr_prdcts) :demand(nmbr_strs, std::vector<int>(nmbr_prdcts)) {};
};
struct MyAction{
  std::vector < std::pair< int, std::vector < int > > > trucks; //[truck->(store, [ product_id -> amount ])] 
  std::vector<int> total_prduc; //[product -> total amount(includes whats stocked and what sent in trucks)]
};
class Node{
protected:
  Node* parent_;
  float success_;
  int nmbr_simls_;
  int depth_;
  virtual bool IamLeaf() = 0;
  // virtual float simulate() = 0;
  // virtual void backpropagate() = 0;
  Node(Node* parent, int depth): parent_{parent}, success_{0}, nmbr_simls_{0}, depth_{depth} {};
public:

  float fitness(int parent_smls, float exploration_factor);

  virtual Node* select( float exploration_factor ) = 0;
  virtual void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* chld_node) = 0;
  virtual void findCurrentTotalDmnd(std::vector<std::vector<int> > &crnt_total_demand, Node* chld_node) = 0;
  virtual void expand(  int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
    int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min) = 0;
};

class WorldTurn: public Node{
private:
  std::list<std::pair<WorldAction, Node*> > children;

  static WorldAction createRandomWorldAction(int nmbr_strs, int nmbr_prdcts, int time,int time_frm, int demand_range,int demand_min, const std::vector<float> &cmltv_demand_prb_dstrbutn);

  bool IamLeaf() override;
  void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* chld_node) override;
  void findCurrentTotalDmnd(std::vector<std::vector<int> > &crnt_total_demand, Node* chld_node) override;
public:
  WorldTurn(Node* parent, int depth):Node{parent, depth} {};
  Node* select(float exploration_factor) override ;
  void expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
    int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min) override ;
};

class MyTurn : public Node{
private:
  std::list<std::pair<MyAction, Node*> > children;

  static MyAction createRandomMyAction(int truck_capacity, int factory_production_limit, const std::vector<int> &wareHouseState, const std::vector< std::vector<int32_t> > &crnt_total_demand);

  bool IamLeaf() override ; 
  void findWarehouseState(std::vector<int32_t> &wareHouseState, Node* chld_node) override ;
  void findCurrentTotalDmnd(std::vector<std::vector<int> > &crnt_total_demand, Node* chld_node) override;

public:
  MyTurn(Node* parent, int depth):Node{parent, depth} {};
  Node* select(float exploration_factor) override ;
  void expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
    int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min) override ;
};

class MonteCarloTreeSearchCpp {
private:
  Node* treeRoot;
  int demand_min_;
  int demand_max_;
  int demand_range_;
  int nmbr_strs_;
  int nmbr_prdcts_;
  int time_frm_;
  int DC_cpcty_;
  int truck_capacity_;
  int factory_production_limit_;
  int nmbr_brnch_wrldTurn_;
  int nmbr_brnch_myTurn_;
  int nmbr_simulations_per_rollout_;
  float exploration_factor_;
  std::vector<int32_t> wareHouseState_;
  std::vector<float> demand_prb_dstrbutn_;
  std::vector<float> cmltv_demand_prb_dstrbutn_;

public:
  MonteCarloTreeSearchCpp(int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,int truck_capacity,
  int factory_production_limit_, int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn,int nmbr_simulations_per_rollout,  float exploration_factor,
   py::array_t<int32_t> wareHouseStatePy, py::array_t<float> demand_prb_dstrbutnPy) ;


  
  void operator () (int number_of_iterations);
};

