#include "MCTS.h"

float Node::fitness(int parent_smls, float exploration_factor){
    float exploit = success_ / nmbr_simls_;
    float explore = exploration_factor * sqrt(log(parent_smls) / this->nmbr_simls_);
    return exploit + explore;
}

bool WorldTurn::IamLeaf() {
  return children.empty();
}
void WorldTurn::findWarehouseState(std::vector<int32_t> &wareHouseState, Node* chld_node)  {
  
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
void WorldTurn::findCurrentTotalDmnd(std::vector< std::vector<int> > &crnt_total_demand, Node* chld_node){
  for(const auto child : this->children){
    if(child.second == chld_node){
      
      int nmbr_strs = crnt_total_demand.size();
      for(int store = 0; store < nmbr_strs ; store++){
        std::transform(crnt_total_demand[store].begin(), crnt_total_demand[store].end(), child.first.demand[store].begin(), crnt_total_demand[store].begin(), std::plus<int>{});
      }
      
      break;
    }
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
WorldAction WorldTurn::createRandomWorldAction(int nmbr_strs, int nmbr_prdcts,int time, int time_frm, int demand_range,int demand_min, const std::vector<float> &cmltv_demand_prb_dstrbutn){
  WorldAction wa(nmbr_strs, nmbr_prdcts);
  for(int store = 0 ; store < nmbr_strs ; store++){
    for(int product = 0 ; product < nmbr_prdcts ; product++){
      float rndm_nmbr_nrml = get_rndm_nmbr_nrml();
      int data_indx = rowMajor4D(store, product, time, 0, nmbr_strs, nmbr_prdcts, time_frm, demand_range);
      wa.demand[store][product] = demand_min + binarySearchFirstLargerIndex(cmltv_demand_prb_dstrbutn, rndm_nmbr_nrml, data_indx, data_indx + demand_range);
    }
  }
  return wa;
}  
void WorldTurn::expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn, 
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
      WorldAction wa = this->createRandomWorldAction(nmbr_strs, nmbr_prdcts, this->depth_/2,time_frm, demand_range, demand_min, cmltv_demand_prb_dstrbutn);
      Node* chld_nd = new MyTurn(this, this->depth_ + 1);
      children.push_back(std::pair<WorldAction, Node*>(wa, chld_nd));
      // chld_nd->expand3re
    }

}



bool MyTurn::IamLeaf() {
  return children.empty();
}
void MyTurn::findWarehouseState(std::vector<int32_t> &wareHouseState, Node* chld_node)  {
  for(const auto child : children){
    if(child.second == chld_node){
      for(const auto truck : child.first.trucks){
        std::transform(wareHouseState.begin(), wareHouseState.end(), truck.second.begin(), wareHouseState.begin(), std::minus<int>{});
      }
      std::transform(wareHouseState.begin(), wareHouseState.end(), child.first.total_prduc.begin(), wareHouseState.begin(), std::plus<int>{});

      break;
    }
  }
  this->parent_->findWarehouseState(wareHouseState, this);
  // if(this->parent_){
  //   this->parent_->findWarehouseState(wareHouseState, this);
  // }
}

void MyTurn::findCurrentTotalDmnd(std::vector< std::vector<int> > &crnt_total_demand, Node* chld_node){
  for(const auto child : this->children){
    if(child.second == chld_node){
      int nmbr_strs = crnt_total_demand.size();

      for(const auto truck: child.first.trucks){
        std::transform(crnt_total_demand[truck.first].begin(), crnt_total_demand[truck.first].end(),truck.second.begin(), crnt_total_demand[truck.first].begin(), std::minus<int>{});
      }

      break;
    }
  }
  this->parent_->findCurrentTotalDmnd(crnt_total_demand, this);
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
MyAction MyTurn::createRandomMyAction(int truck_capacity, int factory_production_limit, const std::vector<int> &wareHouseState, 
const std::vector< std::vector<int32_t> > &crnt_total_demand ){
    /***
        Steps
        1) Find the cummulative deamnd right now, current demand + what wasn't met before
        2) Create trucks for each store such that entire cummulative demand is met (randomly)
        3) Look at each truck which is partially full and reject it with a certain probability 
            !!=> Right now the probability of rejection is just = number of empty slots / total slots
        4) Now that trucks to be sent have been created, look at whether the DC Stock + the factory production limit can meet the demand
            4.1)If it can
                    4.1.1)Send the trucks 
                    4.1.2) Look at remaining factory production power and DC empty stock, and randomly restock
            4.2) If it cannot 
                4.2.1)look at trucks which are going semi filled and eliminate the once that can be entirely eliminated 
                4.2.1) Randomly select which products not to send 
                4.2.2) send the trucks, no option to restock, since factory will already be at its limit. 
    ***/
  int nmbr_strs = crnt_total_demand.size();
  int nmbr_prdcts = crnt_total_demand.at(0).size();
  std::vector<bool> willLastTruckBeSent(nmbr_strs), semiFilledTruckNeeded(nmbr_strs);
  std::vector<int> total_product_store(nmbr_strs), //[store-> demand of all products added]
  product_to_demand(nmbr_prdcts); //[product->entire demand of that product at warehouse]
  int ignored_demand = 0;
  for(int store = 0 ; store < nmbr_strs ; store++){
    total_product_store[store] = std::accumulate(crnt_total_demand[store].begin(), crnt_total_demand[store].end(), 0);
    semiFilledTruckNeeded[store] = total_product_store[store] % truck_capacity;
    float lastTrkUtilizsn = static_cast<float>(total_product_store[store] % truck_capacity) / static_cast<float>(truck_capacity);
    willLastTruckBeSent[store] = get_rndm_nmbr_nrml() <  lastTrkUtilizsn;
  }
  {
    int store = 0;
    ignored_demand = std::accumulate(total_product_store.begin(), total_product_store.end(), 0, 
      [&truck_capacity, &willLastTruckBeSent, &store](int total, int current){
        int ans = total + ( current % truck_capacity ) * ( ! willLastTruckBeSent[store] );
        store++;
        return ans;
      }
    );
  }
  for(int product = 0 ; product < nmbr_prdcts ; product++){
    product_to_demand[product] = std::accumulate(crnt_total_demand.begin(), crnt_total_demand.end(), 0, 
      [&product](int total,const std::vector<int> &stores_demand){
        return total + stores_demand[product];
      }
    );
  }
  std::vector<int> unmet_demand(nmbr_prdcts);
  std::transform(product_to_demand.begin(), product_to_demand.end(), wareHouseState.begin(), unmet_demand.begin(), 
    [](const int product_demand, const int product_in_warehouse){
      return std::max( 0, product_demand - product_in_warehouse );
    }
  );
  int total_unmet_demand = std::accumulate(unmet_demand.begin(), unmet_demand.end(), 0) - ignored_demand ; 
  MyAction my_actn;
  if( total_unmet_demand < factory_production_limit ){
    std::vector<int> unsent_produce = BeggarsAlgorithm( nmbr_prdcts + 1 , factory_production_limit - total_unmet_demand);
    unsent_produce.pop_back();
    //the last index being the number of slots we keep empty
    my_actn.total_prduc = product_to_demand;
    std::transform(my_actn.total_prduc.begin(), my_actn.total_prduc.end(), unsent_produce.begin(), my_actn.total_prduc.begin(), std::plus<int>{});
    for(int store = 0 ; store < nmbr_strs ; store++){
      std::vector<int> send_to_store = crnt_total_demand[store];
      int nmbr_prdcts_to_send = total_product_store[store];
      if( (! willLastTruckBeSent[store]) && semiFilledTruckNeeded[store]){
        int number_of_products_to_be_dropped = total_product_store[store] % truck_capacity;
        nmbr_prdcts_to_send -= number_of_products_to_be_dropped;
        while(number_of_products_to_be_dropped > 0){
          int rndm_prdct = get_rndm_int(0, nmbr_prdcts);
          if(send_to_store[rndm_prdct] > 0){
            send_to_store[rndm_prdct]--;
            my_actn.total_prduc[rndm_prdct]--;
            number_of_products_to_be_dropped--;
          }
        }
      }
      {
        int product = 0;
        int truck_unused = 0;
        // my_actn.trucks.push_back(make_pair(store, std::vector<int>(nmbr_prdcts, 0)));
        // truck_unused = truck_capacity;
        while(nmbr_prdcts_to_send > 0){
          if(send_to_store[product] > 0 && truck_unused > 0){
            int product_shift = std::min(send_to_store[product], truck_unused);
            my_actn.trucks.back().second[product] += product_shift;
            send_to_store[product] -= product_shift;
            truck_unused -= product_shift;
            nmbr_prdcts_to_send -= product_shift;
          }else if(send_to_store[product] == 0 && truck_unused > 0){
            product++;
          }else if(send_to_store[product] > 0 && truck_unused == 0){
            my_actn.trucks.push_back(make_pair(store, std::vector<int>(nmbr_prdcts, 0)));
            truck_unused = truck_capacity;
          }else{
            // send_to_store[product] == 0 && truck_unused == 0
            product++;
            my_actn.trucks.push_back(make_pair(store, std::vector<int>(nmbr_prdcts, 0)));
            truck_unused = truck_capacity;
          }
        }
      }


    }


  }else{

  }
  return my_actn;
}
void MyTurn::expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn_, 
  int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min)  {

  std::vector< std::vector<int32_t> > crnt_total_demand(nmbr_strs, std::vector<int>(nmbr_prdcts, 0));
  this->findCurrentTotalDmnd(crnt_total_demand, this);

}


MonteCarloTreeSearchCpp::MonteCarloTreeSearchCpp(int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,int truck_capacity,
  int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn,int nmbr_simulations_per_rollout, int factory_production_limit,float exploration_factor,
   py::array_t<int32_t> wareHouseStatePy, py::array_t<float> demand_prb_dstrbutnPy) : 
    // widthOfFirstLevel_{widthOfFirstLevel},
    demand_min_{demand_min},
    demand_max_{demand_max},
    nmbr_strs_{nmbr_strs},
    nmbr_prdcts_{nmbr_prdcts},
    time_frm_{time_frm},
    DC_cpcty_{DC_cpcty},
    truck_capacity_{truck_capacity},
    factory_production_limit_{factory_production_limit},
    nmbr_brnch_wrldTurn_{nmbr_brnch_wrldTurn},
    nmbr_brnch_myTurn_{nmbr_brnch_myTurn},
    nmbr_simulations_per_rollout_{nmbr_simulations_per_rollout},
    // max_nmbr_trucks_{max_nmbr_trucks},
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
    
    // if(widthOfFirstLevel == -1){
    //   widthOfFirstLevel_ = demand_range_;
    // }else if (widthOfFirstLevel <= demand_range_ && widthOfFirstLevel > 0){
    //   widthOfFirstLevel_ = widthOfFirstLevel;
    // }else{
    //   throw std::runtime_error("Width of first level not sent properly. It should be -1 for max, or in [1, max]");
    // }
    
    treeRoot = new WorldTurn(nullptr, 0);
    
}


void MonteCarloTreeSearchCpp::operator () (int number_of_iterations){
  while(number_of_iterations--){
    Node* node = treeRoot->select(this->exploration_factor_);
    node->expand( this->nmbr_brnch_wrldTurn_, this->nmbr_brnch_myTurn_, this->wareHouseState_,this->cmltv_demand_prb_dstrbutn_, this->nmbr_strs_, 
      this->nmbr_prdcts_, this->time_frm_, this->demand_range_, this->demand_min_ );
  }
}


PYBIND11_MODULE(PyMCTS, module_handle) {
  module_handle.doc() = "I'm a docstring hehe";
  py::class_<MonteCarloTreeSearchCpp>(
			module_handle, "MonteCarloTreeSearch"
			).def(py::init<int,int,int,int,int,int,int,int,int,int,int,float,py::array_t<int32_t> , py::array_t<float> >())
    ;
}