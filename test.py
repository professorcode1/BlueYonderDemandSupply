from MonteCarloTreeSearch_Source.build.PyMCTS import MonteCarloTreeSearch
from  TestCase import TestCase
import numpy as np
if __name__ == "__main__":
    # MonteCarloTreeSearch(widthOfFirstLevel,int demand_min,int demand_max,int nmbr_strs,int nmbr_prdcts,int time_frm,int DC_cpcty,int nmbr_brnch_wrldTurn, 
    # int nmbr_brnch_myTurn, py::array_t<int32_t> wareHouseStatePy, py::array_t<float> demand_prb_dstrbutnPy)
    # print(PyMCTS._FuncPtr)
    stores = 5
    products = 7
    time_frm = 90
    DC_cpcty = 50
    tc = TestCase( stores, products, DC_capacity = DC_cpcty )
    demand_prb_dstrbutnPy = tc.getData(1000, simplex_reduction_factor = 0.3)
    nmbr_brnch_wrldTurn = 10
    nmbr_brnch_myTurn = 10
    MonteCarloTreeSearch(100, 10, 400, stores, products, time_frm,DC_cpcty, nmbr_brnch_wrldTurn, nmbr_brnch_myTurn,np.zeros(products, dtype=np.int32), demand_prb_dstrbutnPy)
    