#include "MCTS.h"

float Node::fitness(int parent_smls, float exploration_factor){
    float exploit = success_ / nmbr_simls_;
    float explore = exploration_factor * sqrt(log(parent_smls) / this->nmbr_simls_);
    return exploit + explore;
}

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
    /***
        The expand function  just create nmbr_brnch_wrldTurn simulation, with the probability of each simulation same as in the 
        demand probability distribution.
        In the future, it can be changed to give preference to outlier possible outcomes as well
        Like always make sure that very low demand and very high demands are simulated
        Or like divide the demand range into buckets and make sure each bucket has one representative.
    ***/
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
      Node* chld_nd = new MyTurn(this, this->depth_ + 1);
      children.push_back(std::pair<WorldAction, Node*>(wa, chld_nd));
      // chld_nd->expand3re
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
    if(this->IamLeaf()){
        throw std::runtime_error("A MyTurn Node is a leaf node");
    }
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
    /***
        Steps
        1) Find the cummulative deamnd right now, current demand + what wasn't met before
        2) Create trucks for each store such that entire cummulative demand is met
        3) Look at each truck which is partially full and reject it with a certain probability 
            !!=> Right now the probability of rejection is just = number of empty slots / total slots
            but it can be made more heuristically intelligent in the furute by maiting a priority queue
            where if a products demand hasn't been met at a store for too long, it has a higher priority of being met
            even if it means the truck will have to be used unoptimally.
        4) Now that trucks to be sent have been created, look at whether the DC Stock + the factory production limit can meet this demand
            4.1)If it can
                    4.1.1)Send the trucks 
                    4.1.2) Look at remaining factory production power and DC empty stock, and randomly restock
                                !!=> Ine the future this random restock can be made  heuristically intelligent by looking at 
                                        future predictions of demand 
            4.2) If it cannot 
                4.2.1) Randomly select which products not to send 
                    !!=> this can be made intelligent by reconsidering sending all trucks and selecting demand to ignore such that trucks
                            are sent optimally
                4.2.2) send the trucks, no option to restock, since factory will already be at its limit. 
    ***/
}


MonteCarloTreeSearchCpp::MonteCarloTreeSearchCpp(int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,
  int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn,int nmbr_simulations_per_rollout, float exploration_factor,
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
			).def(py::init<int,int,int,int,int,int,int,int,int,float,py::array_t<int32_t> , py::array_t<float> >())
    ;
}