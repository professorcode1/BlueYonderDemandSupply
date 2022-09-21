#include "MCTS.h"


bool WorldTurn::IamLeaf() {
  return children.empty();
}
void WorldTurn::findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node)  {
  
  // for(const auto child : children){
  //   if(child.second == node){
  //     for(const auto store: child.first.demand){
  //       std::transform(wareHouseState.begin(), wareHouseState.end(), store.begin(), wareHouseState.begin(), std::plus<int>{});
  //     }
  //   }
  // }
  //damn bad logic above, the world just puts a demand, it doesn't do anything to the warehouse 
  if(this->parent_){
    this->parent_->findWarehouseState(wareHouseState, this);
  }
}
Node* WorldTurn::select(float exploration_factor)  {
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
  return data_->second->select(exploration_factor); 
}
void WorldTurn::expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
  int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min)  {
  
  if(this->parent_){
    this->parent_->findWarehouseState(wareHouseState, this);
  }
  for(int iter_brnch = 0 ; iter_brnch < nmbr_brnch_wrldTurn ; iter_brnch++){
    WorldAction wa(nmbr_strs, nmbr_prdcts);
    for(int store = 0 ; store < nmbr_strs ; store++){
      for(int product = 0 ; product < nmbr_prdcts ; product++){
        float rndm_nmbr_nrml = static_cast<float>(static_cast<int64_t>(rndm_nmbr_gnrt()) - static_cast<int64_t>(std::mt19937::min())) / static_cast<float>(static_cast<int64_t>(std::mt19937::max()) - static_cast<int64_t>(std::mt19937::min()));
        int data_indx = rowMajor4D(store, product, this->depth_/2, 0, nmbr_strs, nmbr_prdcts, time_frm, demand_range);
        wa.demand[store][product] = demand_min + binarySearchFirstLargerIndex(cmltv_demand_prb_dstrbutn_, rndm_nmbr_nrml, data_indx, data_indx + demand_range);
      }
    }
    children.push_back(std::pair<WorldAction, Node*>(wa, new MyTurn(this, this->depth_ + 1)));
  }
}



bool MyTurn::IamLeaf() {
  return children.empty();
}
void MyTurn::findWarehouseState(std::vector<int32_t> &wareHouseState, Node* node)  {
  for(const auto child : children){
    if(child.second == node){
      for(const auto truck : child.first.trucks){
        std::transform(wareHouseState.begin(), wareHouseState.end(), truck.second.begin(), wareHouseState.begin(), std::minus<int>{});
      }
      std::transform(wareHouseState.begin(), wareHouseState.end(), child.first.total_prduc.begin(), wareHouseState.begin(), std::plus<int>{});
    }
  }
  if(this->parent_){
    this->parent_->findWarehouseState(wareHouseState, this);
  }
}
Node* MyTurn::select(float exploration_factor)  {
  if(this->IamLeaf())
    return this;
  Node* fitest_chld = nullptr;
  float crnt_bst_fitness = std::numeric_limits<float>::lowest();
  for(const auto data_ : children ){
    float crnt_fitness = data_.second->fitness(this->nmbr_simls_, exploration_factor);
    if( crnt_fitness > crnt_bst_fitness ){
      crnt_bst_fitness = crnt_fitness;
      fitest_chld = data_.second;
    }
  }
  return fitest_chld->select(exploration_factor);
}
void MyTurn::expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> &wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
  int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min)  {

}




void MonteCarloTreeSearchCpp::operator () (int number_of_iterations){
  while(number_of_iterations--){
    Node* node = treeRoot->select(this->exploration_factor_);
    {
      std::vector<int32_t> wareHouseStateCpy(wareHouseState_);
      node->expand( this->nmbr_brnch_wrldTurn_, this->nmbr_brnch_myTurn_, wareHouseStateCpy,cmltv_demand_prb_dstrbutn_, this->nmbr_strs_, this->nmbr_prdcts_, 
        this->time_frm_, this->demand_range_, this->demand_min_ );
    }
  }
}


PYBIND11_MODULE(PyMCTS, module_handle) {
  module_handle.doc() = "I'm a docstring hehe";
  py::class_<MonteCarloTreeSearchCpp>(
			module_handle, "MonteCarloTreeSearch"
			).def(py::init<int,int,int,int,int,int,int,int,int,int,int,float,py::array_t<int32_t> , py::array_t<float> >())
    ;
}