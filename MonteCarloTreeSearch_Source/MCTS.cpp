#include "MCTS.h"

float Node::fitness(int parent_smls, float exploration_factor){
  if(nmbr_simls_ == 0)
    // return std::numeric_limits<float>::max();
    throw std::runtime_error("Zero simulations but fitness evaluated");
  float exploit = success_ / nmbr_simls_;
  float explore = exploration_factor * sqrt(log(parent_smls) / this->nmbr_simls_);
  return exploit + explore;
}

WorldTurn::~WorldTurn(){
  for(auto &child : this->children)
    delete child.second;
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
  if(this->parent_){
    this->parent_->findCurrentTotalDmnd(crnt_total_demand, this);
  }
}
void WorldTurn::findOverAllTotalDmnd(std::vector< std::vector<int> > &oval_total_demand, Node* chld_node){
  for(const auto child : this->children){
    if(child.second == chld_node){
      
      int nmbr_strs = oval_total_demand.size();
      for(int store = 0; store < nmbr_strs ; store++){
        std::transform(oval_total_demand[store].begin(), oval_total_demand[store].end(), child.first.demand[store].begin(), oval_total_demand[store].begin(), std::plus<int>{});
      }
      
      break;
    }
  }
  if(this->parent_){
    this->parent_->findCurrentTotalDmnd(oval_total_demand, this);
  }
}
void WorldTurn::findUnFldTruckUsages(std::vector<int> &usage, Node* chld_node, int truck_capacity) {
  if(this->parent_){
    this->parent_->findUnFldTruckUsages(usage, this, truck_capacity);
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
  std::list<std::pair<WorldAction, MyTurn*> >::iterator data_ = children.begin();
  advance(data_, indexOfNodeToExplore);
  return data_->second->select(exploration_factor); 
}
WorldAction Node::createRandomWorldAction(int nmbr_strs, int nmbr_prdcts,int time, int time_frm, int demand_range,int demand_min, const std::vector<float> &cmltv_demand_prb_dstrbutn){
  WorldAction wa(nmbr_strs, nmbr_prdcts);
  for(int store = 0 ; store < nmbr_strs ; store++){
    for(int product = 0 ; product < nmbr_prdcts ; product++){
      float rndm_nmbr_nrml = get_rndm_nmbr_nrml();
      int data_indx = rowMajor4D(store, product, time, 0, nmbr_strs, nmbr_prdcts, time_frm, demand_range);
      wa.demand[store][product] = demand_min + binarySearchFirstLargerIndex<float>(cmltv_demand_prb_dstrbutn, rndm_nmbr_nrml, data_indx, data_indx + demand_range);
    }
  }
  return wa;
}  
void WorldTurn::expand( int nmbr_brnch_wrldTurn, int nmbr_brnch_myTurn, std::vector<int32_t> wareHouseState, const std::vector<float> &cmltv_demand_prb_dstrbutn, 
  int nmbr_strs, int nmbr_prdcts, int time_frm, int demand_range, int demand_min, int truck_capacity, int factory_production_limit, int DC_cpcty)  {
    /***
        The expand function  just create nmbr_brnch_wrldTurn simulation, with the probability of each simulation same as in the 
        demand probability distribution.
        In the future, it can be changed to give preference to outlier possible outcomes as well
        Like always make sure that very low demand and very high demands are simulated
        Or like divide the demand range into buckets and make sure each bucket has one representative.
    ***/
    std::vector<std::vector<int> > crnt_total_demand(nmbr_strs, std::vector<int>(nmbr_prdcts, 0));
    if(this->parent_){
      this->parent_->findWarehouseState(wareHouseState, this);
      this->parent_->findCurrentTotalDmnd(crnt_total_demand, this);
    }
    for(int iter_brnch = 0 ; iter_brnch < nmbr_brnch_wrldTurn ; iter_brnch++){
      WorldAction wa = this->createRandomWorldAction(nmbr_strs, nmbr_prdcts, this->depth_/2,time_frm, demand_range, demand_min, cmltv_demand_prb_dstrbutn);
      MyTurn* chld_nd = new MyTurn(this, this->depth_ + 1);
      children.push_back(std::pair<WorldAction, MyTurn*>(wa, chld_nd));
      
      int nmbr_strs = crnt_total_demand.size();
      for(int store = 0; store < nmbr_strs ; store++){
        std::transform(crnt_total_demand[store].begin(), crnt_total_demand[store].end(), wa.demand[store].begin(), crnt_total_demand[store].begin(), std::plus<int>{});
      }
      chld_nd->expand(wareHouseState, crnt_total_demand, nmbr_brnch_myTurn, truck_capacity, factory_production_limit, DC_cpcty);
      for(int store = 0; store < nmbr_strs ; store++){
        std::transform(crnt_total_demand[store].begin(), crnt_total_demand[store].end(), wa.demand[store].begin(), crnt_total_demand[store].begin(), std::minus<int>{});
      }
    }

}
void WorldTurn::simulate(const std::vector<int> &wareHouseState, const std::vector<std::vector<int> > &crnt_total_demand, int truck_capacity,
  int nmbr_simulations_per_rollout, int time_frm,int factory_production_limit , int DC_cpcty, int demand_range, int demand_min, 
  const std::vector<float> &cmltv_demand_prb_dstrbutn ){
  int nmbr_strs = crnt_total_demand.size();
  int nmbr_prdcts = crnt_total_demand[0].size();
  std::vector<int> unfilled_trucks_usage;
  std::vector<std::vector< int> > oval_total_demand(nmbr_strs, std::vector<int>(nmbr_prdcts, 0));
  this->findUnFldTruckUsages(unfilled_trucks_usage, this, truck_capacity);
  this->findOverAllTotalDmnd(oval_total_demand, this);
  for(int simulation_iter = 0 ; simulation_iter < nmbr_simulations_per_rollout ; simulation_iter++){
    int current_depth = this->depth_;
    std::vector<std::vector<int> > sim_crnt_total_demand = crnt_total_demand;
    std::vector<std::vector<int> > sim_oval_total_demand = oval_total_demand;
    std::vector<int> sim_unfilled_trucks_usage = unfilled_trucks_usage;
    std::vector<int> sim_wareHouseState = wareHouseState;

    while(current_depth < 2 * time_frm){
      MyAction my_actn = this->createRandomMyAction(truck_capacity, factory_production_limit, DC_cpcty, sim_wareHouseState, sim_crnt_total_demand );
      WorldAction wrld_actn = this->createRandomWorldAction(nmbr_strs, nmbr_prdcts, current_depth / 2, time_frm, demand_range, demand_min, cmltv_demand_prb_dstrbutn);
      
      

      current_depth += 2;
    }

  }
}

MyTurn::~MyTurn(){
  for(auto &child: this->children)
    delete child.second;
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
void MyTurn::findOverAllTotalDmnd(std::vector< std::vector<int> > &oval_total_demand, Node* chld_node){
  this->parent_->findCurrentTotalDmnd(oval_total_demand, this);
}
void MyTurn::findUnFldTruckUsages(std::vector<int> &usage, Node* chld_node, int truck_capacity){
  for(const auto child : this->children){
    if(child.second == chld_node){
      // int nmbr_strs = crnt_total_demand.size();

      for(const auto truck: child.first.trucks){
        int usage_val = std::accumulate(truck.second.begin(), truck.second.end(), 0);
        if( usage_val != truck_capacity){
          usage.push_back(usage_val);
        }
      }

      break;
    }
  }
  this->parent_->findUnFldTruckUsages(usage, this, truck_capacity);
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
void Node::createRandomMyActionSubRoutine_GenerateTuckingForStore(const std::vector<bool> &willLastTruckBeSent, 
  const std::vector<bool> &semiFilledTruckNeeded, int nmbr_prdcts_to_send, const int truck_capacity, const int nmbr_prdcts, std::vector<int> &send_to_store,
  MyAction &my_actn, const int store){
  if( (! willLastTruckBeSent[store]) && semiFilledTruckNeeded[store]){
    int number_of_products_to_be_dropped = nmbr_prdcts_to_send % truck_capacity;
    nmbr_prdcts_to_send -= number_of_products_to_be_dropped;
    //this horribly inefficient method can be imporved by sampling number_of_products_to_be_dropped rndm nmbrs b/w [0, total number of products ) 
    //and using that to drop products 
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
int Node::createRandomMyActionSubRoutine_LiabilityTrkVc(std::vector<bool> &willLastTruckBeSent, const std::vector<int> &total_product_store,
  int nmbr_strs, int truck_capacity ){
  int ignored_product = 0;
  //given a willLastTruckBeSent, randomly drop the once which are being sent (i.e evaluates random bool if willLastTruckBeSent is true)
  for(int store = 0 ; store < nmbr_strs ; store++){
    float lastTrkUtilizsn = static_cast<float>(total_product_store[store] % truck_capacity) / static_cast<float>(truck_capacity);
    willLastTruckBeSent[store] = willLastTruckBeSent[store] && (get_rndm_nmbr_nrml() <  lastTrkUtilizsn);
    ignored_product += willLastTruckBeSent[store] * (total_product_store[store] % truck_capacity);
  }
  return ignored_product;
}
MyAction Node::createRandomMyAction(int truck_capacity, int factory_production_limit, int DC_cpcty, const std::vector<int> &wareHouseState, 
const std::vector< std::vector<int32_t> > &crnt_total_demand ){
    /***
        Steps
        1) Find the cummulative deamnd right now, current demand + what wasn't met before
        2) Find what part of this demand needs to be generated by the factory 
        3) Find out the total number of products that will be sent in unfilled trucks if the entire demand can be met = liability_val
        4) If demand can be met by generation from the factory 
           Then decide which trucks not to send and send 
        5)else if, see if (demand - liability_val) can be met 
          If it can, randomly drop trucks till it dropped trucks prouct doesnt exceed liability_val
          Now drop the remaining trucks randomly with prb prp to the emptiness
        6) else 
          Randomly reduce (store, product) value till it can be met
          make the trucks
          Drop the liability trucks randomly 
    ***/
  int nmbr_strs = crnt_total_demand.size();
  int nmbr_prdcts = crnt_total_demand.at(0).size();
  std::vector<bool> semiFilledTruckNeeded(nmbr_strs);
  std::vector<int> total_product_store(nmbr_strs), //[store-> demand of all products added]
  product_to_demand(nmbr_prdcts); //[product->entire demand of that product at warehouse]
  int liability_val = 0;
  for(int store = 0 ; store < nmbr_strs ; store++){
    total_product_store[store] = std::accumulate(crnt_total_demand[store].begin(), crnt_total_demand[store].end(), 0);
    semiFilledTruckNeeded[store] = total_product_store[store] % truck_capacity;
    liability_val += total_product_store[store] % truck_capacity;
  }
  for(int product = 0 ; product < nmbr_prdcts ; product++){
    product_to_demand[product] = std::accumulate(crnt_total_demand.begin(), crnt_total_demand.end(), 0, 
      [&product](int total,const std::vector<int> &stores_demand){
        return total + stores_demand[product];
      }
    );
  }
  std::vector<int> fctry_gnrt_demand(nmbr_prdcts);
  int DC_prdcts_used = 0;
  int DC_ttl_prds = 0;
  std::transform(product_to_demand.begin(), product_to_demand.end(), wareHouseState.begin(), fctry_gnrt_demand.begin(), 
    [&DC_prdcts_used, &DC_ttl_prds](const int product_demand, const int product_in_warehouse){
      DC_ttl_prds += product_in_warehouse;
      DC_prdcts_used += std::min( product_in_warehouse , product_demand );
      return std::max( 0, product_demand - product_in_warehouse );
    }
  );
  int total_fctry_gnrt_demand = std::accumulate(fctry_gnrt_demand.begin(), fctry_gnrt_demand.end(), 0); 
  MyAction my_actn;
  std::vector<bool> willLastTruckBeSent(nmbr_strs, true);
  if( ( total_fctry_gnrt_demand - liability_val ) <= factory_production_limit ){
    //5
    if( total_fctry_gnrt_demand > factory_production_limit ){
      int dropped_truk_val = 0;
      while( dropped_truk_val < ( total_fctry_gnrt_demand - factory_production_limit ) ){
        int drpd_str = get_rndm_int(0, nmbr_strs);
        if(willLastTruckBeSent[drpd_str]){
          willLastTruckBeSent[drpd_str] = false;
          dropped_truk_val += total_product_store[drpd_str] % truck_capacity;
        }
      }
    }
  }else{
    //6
    [&]{
      std::vector<int> random_numbers = get_rndm_ints(0, total_fctry_gnrt_demand, total_fctry_gnrt_demand - factory_production_limit );
      sort(random_numbers.begin(), random_numbers.end());
      int low ,high = 0;
      for(int store = 0 ; store < nmbr_strs ; store++){
        for(int product = 0 ; product < nmbr_prdcts ; product++){
          low = high;
          high = crnt_total_demand[store][product] + low;
          if(low >= random_numbers.back())
            return ;
          int product_drop = 0;
          for(int strt_indx = binarySearchFirstLargerIndex<int>( random_numbers, low, 0, random_numbers.size()); strt_indx < random_numbers.size() ; strt_indx++){
            if(random_numbers[strt_indx] >= high)
              break;
            product_drop++;
          }

          fctry_gnrt_demand[product] -= product_drop;
        }
      }
      total_fctry_gnrt_demand = factory_production_limit;
    }();

  }
  int ignored_product = createRandomMyActionSubRoutine_LiabilityTrkVc( willLastTruckBeSent, total_product_store, nmbr_strs, truck_capacity);  
      
  int fctry_pwr_srpls = factory_production_limit - ( total_fctry_gnrt_demand - ignored_product );
  int DC_cpcty_srpls = DC_cpcty - ( DC_ttl_prds - DC_prdcts_used );
  std::vector<int> unsent_produce = BeggarsAlgorithm( nmbr_prdcts + 1 , std::min(fctry_pwr_srpls, DC_cpcty_srpls) );
  unsent_produce.pop_back();
  //the last index being the number of slots we keep empty
  my_actn.total_prduc.resize(nmbr_prdcts);
  std::transform(fctry_gnrt_demand.begin(), fctry_gnrt_demand.end(), unsent_produce.begin(), my_actn.total_prduc.begin(), std::plus<int>{});
  for(int store = 0 ; store < nmbr_strs ; store++){
    std::vector<int> send_to_store = crnt_total_demand[store];
    int nmbr_prdcts_to_send = total_product_store[store];
    createRandomMyActionSubRoutine_GenerateTuckingForStore(willLastTruckBeSent, semiFilledTruckNeeded, nmbr_prdcts_to_send, truck_capacity, 
      nmbr_prdcts, send_to_store, my_actn, store);
  }
  return my_actn;
}
void MyTurn::expand(const std::vector<int> &wareHouseState, const std::vector<std::vector<int> > &crnt_total_demand, int nmbr_brnch_myTurn, int truck_capacity,
int factory_production_limit, int DC_cpcty ){

  for(int iter_brnch = 0 ; iter_brnch < nmbr_brnch_myTurn ; iter_brnch++){
    MyAction ma = this->createRandomMyAction(truck_capacity, factory_production_limit, DC_cpcty, wareHouseState, crnt_total_demand);
    WorldTurn *chld_nd = new WorldTurn(this, this->depth_ + 1);
    children.push_back(std::pair<MyAction, WorldTurn*>(ma, chld_nd));
  }

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

MonteCarloTreeSearchCpp::~MonteCarloTreeSearchCpp(){
  delete this->treeRoot;
}
void MonteCarloTreeSearchCpp::MainLoop () {
    WorldTurn* node = static_cast<WorldTurn* >(treeRoot->select(this->exploration_factor_));
    node->expand( this->nmbr_brnch_wrldTurn_, this->nmbr_brnch_myTurn_, this->wareHouseState_,this->cmltv_demand_prb_dstrbutn_, this->nmbr_strs_, 
      this->nmbr_prdcts_, this->time_frm_, this->demand_range_, this->demand_min_ , this->truck_capacity_, this->factory_production_limit_, 
      this->DC_cpcty_);
}


PYBIND11_MODULE(PyMCTS, module_handle) {
  module_handle.doc() = "I'm a docstring hehe";
  py::class_<MonteCarloTreeSearchCpp>(
			module_handle, "MonteCarloTreeSearch"
			).def(py::init<int,int,int,int,int,int,int,int,int,int,int,float,py::array_t<int32_t> , py::array_t<float> >())
      .def("__call__", &MonteCarloTreeSearchCpp::MainLoop)
    ;
}