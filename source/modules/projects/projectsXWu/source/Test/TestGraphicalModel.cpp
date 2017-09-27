//#include "StdAfx.h"
////---------------------------------------------------------------------------
//#include "Test/PCITestCentr.h"
////---------------------------------------------------------------------------
//#include "Timer.h"
////---------------------------------------------------------------------------
//
//namespace {
//    int DR_FRAME = 0;
//}
//
//#include "Util/TrimeshStatic.h"
//#include "GraphicalModel/GraphicalModel.hpp"
//#include "Util/MRF/mrf.h"
//#include "Util/MRF/GCoptimization.h"
//
//const int sizeX = 50;
//const int sizeY = 50;
//const int numLabels = 20;
//MRF::CostVal DD[sizeX*sizeY*numLabels];
//MRF::CostVal VV[numLabels*numLabels];
//extern MRF::CostVal fnCost(int pix1, int pix2, int i, int j);
//extern EnergyFunction* generate_DataFUNCTION_SmoothGENERAL_FUNCTION();
//
//typedef double ValueType;
//typedef size_t IndexType;
//typedef size_t LabelType;
//typedef GraphicalSpace<IndexType, LabelType> SpaceType;
//typedef GraphicalModel<ValueType, SpaceType> GM;
//typedef GraphicalFactor<GM> FactorType;
//typedef ExplicitFunction<ValueType, IndexType, LabelType> ExplicitFunctionType;
//
//void OptimizeEnergySwap( EnergyFunction * energy )
//{
//    MRF* mrf;
//    mrf = new Swap(sizeX, sizeY, numLabels, energy);
//    mrf->initialize();
//    mrf->clearAnswer();
//
//    MRF::EnergyVal E;
//    float t,tot_t;
//    int iter;
//
//    debugOutput << "\n*******Started graph-cuts swap *****\n";
//    E = mrf->totalEnergy();
//    debugOutput << "Energy at the Start= "
//        << (float)E << " (" << (float)mrf->smoothnessEnergy()
//        << ", " << (float)mrf->dataEnergy() << ")\n";
//
//    tot_t = 0;
//    for (iter=0; iter<8; iter++) {
//        mrf->optimize(1, t);
//
//        E = mrf->totalEnergy();
//        tot_t = tot_t + t ;
//        debugOutput << "energy = " << (float)E << ", (" << tot_t << " secs)\n";
//    }
//
//    delete mrf;
//}
//
//class ExampleTestFunction
//    : public GraphicalFunction<ValueType, IndexType, LabelType>
//{
//public:
//    typedef boost::shared_ptr< ExampleTestFunction > Ptr;
//    typedef boost::shared_ptr< const ExampleTestFunction > ConstPtr;
//
//public:
//    ValueType operator()(
//        IndexType variable1, IndexType variable2,
//        LabelType state1, LabelType state2
//        )
//    {
//        return (ValueType)((
//            variable1*(state1+1)*(state2+2) +
//            variable2*state1*state2*variable1 -
//            2*state1*state2*variable1
//            ) % 100) / 10;
//    }
//};
//
//EnergyFunction* generate_energy_graphical_model()
//{
//    size_t spaceshape[] = {sizeX, sizeY};
//    SpaceType::Ptr space (new SpaceType(spaceshape, spaceshape+2, numLabels));
//    GM::Ptr gm(new GM(space));
//
//    for(size_t variable = 0; variable < gm->numberOfVariables(); ++variable) {
//        const size_t numlabel = gm->numberOfLabels(variable);
//        const size_t shape[] = { numlabel };
//        ExplicitFunctionType::Ptr functPtr (new ExplicitFunctionType(shape, shape + 1));
//        ExplicitFunctionType& f = *functPtr;
//        for(size_t state = 0; state < numlabel; ++state) {
//            f(state) = ((variable*state + state + variable) % 30) / ((MRF::CostVal) 3);
//        }
//        FactorType::Ptr factorPtr (new FactorType(gm));
//        size_t variableIndex[] = { variable };
//        factorPtr->BindFunction(functPtr, variableIndex, variableIndex+1);
//        gm->addFactor(factorPtr);
//    }
//
//    /* allocating these high order factors can be extremely SLOW! */
//
//    //for(size_t variable1 = 0; variable1 < gm->numberOfVariables(); ++variable1) {
//    //    for(size_t variable2 = variable1 + 1; variable2 < gm->numberOfVariables(); ++variable2) {
//    //        ExampleTestFunction::Ptr functPtr (new ExampleTestFunction);
//    //        FactorType::Ptr factorPtr (new FactorType(gm));
//    //        size_t variableIndex[] = { variable1, variable2 };
//    //        factorPtr->BindFunction(functPtr, variableIndex, variableIndex+2);
//    //        gm->addFactor(factorPtr);
//    //    }
//    //}
//
//    //for(size_t variable1 = 0; variable1 < gm->numberOfVariables(); ++variable1) {
//    //    for(size_t variable2 = variable1 + 1; variable2 < gm->numberOfVariables(); ++variable2) {
//    //        const size_t shape[] = {
//    //            gm->numberOfLabels(variable1),
//    //            gm->numberOfLabels(variable2),
//    //        };
//    //        ExplicitFunctionType::Ptr functPtr (new ExplicitFunctionType(shape, shape + 2));
//    //        ExplicitFunctionType& f = *functPtr;
//    //        for(size_t state1 = 0; state1 < gm->numberOfLabels(variable1); ++state1) {
//    //            for(size_t state2 = 0; state2 < gm->numberOfLabels(variable2); ++state2) {
//    //                f(state1, state2) =
//    //                    (ValueType)((variable1*(state1+1)*(state2+2) + variable2*state1*state2*variable1
//    //                    - 2*state1*state2*variable1) % 100) / 10;
//    //            }
//    //        }
//    //        FactorType::Ptr factorPtr (new FactorType(gm));
//    //        size_t variableIndex[] = { variable1, variable2 };
//    //        factorPtr->BindFunction(functPtr, variableIndex, variableIndex+2);
//    //        gm->addFactor(factorPtr);
//    //    }
//    //}
//
//    for (size_t fi = 0; fi < gm->numberOfFactors(); ++fi) {
//        FactorType& factor = *(*gm)[fi];
//        std::vector<IndexType> indices = factor.variableIndices();
//        size_t order = indices.size();
//        size_t fsz1 = 0, fsz2 = 0;
//
//        if (order == 1) {
//            const size_t variable = indices[0];
//            const size_t numlabel = gm->numberOfLabels(variable);
//            for(size_t state = 0; state < numlabel; ++state) {
//                ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor.getFunction()));
//                DD[variable*numlabel + state] = f(state);
//            }
//            ++fsz1;
//        } else if (order == 2) {
//            const size_t variable1 = indices[0], variable2 = indices[1];
//            const size_t numlabel1 = gm->numberOfLabels(variable1), numlabel2 = gm->numberOfLabels(variable2);
//
//            /* dimension is wrong here */
//
//            //for(size_t state1 = 0; state1 < numlabel1; ++state1) {
//            //    for(size_t state2 = 0; state2 < numlabel2; ++state2) {
//            //        ExampleTestFunction& f = *(boost::dynamic_pointer_cast<ExampleTestFunction>(factor.getFunction()));
//            //        VV[variable1+numlabel1*variable2] = VV[variable2+numlabel2*variable1]
//            //        = f(variable1, variable2, state1, state2);
//            //    }
//            //}
//
//            //for(size_t state1 = 0; state1 < numlabel1; ++state1) {
//            //    for(size_t state2 = 0; state2 < numlabel2; ++state2) {
//            //        ExplicitFunctionType& f = *(boost::dynamic_pointer_cast<ExplicitFunctionType>(factor.getFunction()));
//            //        VV[variable1+numlabel1*variable2] = VV[variable2+numlabel2*variable1]
//            //        = f(state1, state2);
//            //    }
//            //}
//
//            ++fsz2;
//        }
//    }
//
//    DataCost *data         = new DataCost(DD);
//    SmoothnessCost *smooth = new SmoothnessCost(VV);
//    //SmoothnessCost *smooth = new SmoothnessCost(fnCost);
//    EnergyFunction *energy    = new EnergyFunction(data,smooth);
//
//    return energy;
//}
//
//void PCITestCentr::TestGraphicalModel(void)
//{
//    int seed = 1124285485;
//    srand(seed);
//
//    EnergyFunction *energy;
//
//    /* NOTICE: different run can produce different minimization */
//
//    energy = generate_DataFUNCTION_SmoothGENERAL_FUNCTION();
//    OptimizeEnergySwap(energy);
//    delete energy;
//
//    energy = generate_energy_graphical_model();
//    OptimizeEnergySwap(energy);
//    delete energy;
//}
