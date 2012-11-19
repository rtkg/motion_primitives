#include <acado_toolkit.hpp>
#include <string>
#include <deque>
#include <assert.h>
#include <math.h>
#include <iostream>

int main(int argc, char **argv)
{
  USING_NAMESPACE_ACADO

    //KKT tolerance
    const double KKT_tol = 1e-4;

  //READ THE COMMAND LINE
  std::deque<std::string> args(argv + 1, argv + argc + !argc);
  std::string path=args[0];
  args.pop_front();

  //  const int nBF=atoi(args[0].c_str());
  //  args.pop_front();
  //  const int nD=(int)args.size();
  //  int nS=0;
  //  for(int i=0; i<nD;i++)
  //    nS+=atoi(args[i].c_str());


  //READ DATA

  Matrix X = readFromFile((path+"kinematics.dat").c_str()); // X(:,0)=q, X(:,1)=dq, X(:,2)=ddq
  Vector F = readFromFile((path+"force.dat").c_str()); 
  Vector lb = readFromFile((path+"lb.dat").c_str()); 
  Vector ub = readFromFile((path+"ub.dat").c_str()); 
  Vector pI = readFromFile((path+"pI.dat").c_str()); 

  unsigned int nS=F.getDim();
  assert(nS == X.getNumRows());
  unsigned int nC=9;

  //PARAMETER & OBJECTIVE FUNCTION
  Parameter p(nC,(unsigned int)1);
  Expression B(nS, nC);  

  for (unsigned int i=0; i < nS; i++)
    {
      B(i,0)=X(i,0);
      B(i,1)=X(i,1);
      B(i,2)=X(i,2);
      B(i,3)=((2*p(4)*X(i,1)).getExp()-1.0)/((2*p(4)*X(i,1)).getExp()+1.0)-((2*p(5)*X(i,1)).getExp()-1.0)/((2*p(5)*X(i,1)).getExp()+1.0);
      B(i,4)=0.0;
      B(i,5)=0.0;
      B(i,6)=((2*p(7)*X(i,1)).getExp()-1.0)/((2*p(7)*X(i,1)).getExp()+1.0);
      B(i,7)=0.0;
      B(i,8)=X(i,1);
    }

  Expression f;
  f<<B*p-F;

  NLP nlp;
  nlp.minimize(0.5*f.transpose()*f);
  //  nlp.subjectTo(A*S*p <= b);
  nlp.subjectTo(lb <= p <= ub);

  //  if(beq.getDim() > 0)
  //    nlp.subjectTo(Aeq*S*p == beq);

  //ALGORITHM 
  ParameterEstimationAlgorithm algorithm(nlp);
  VariablesGrid initial_guess(nC,0.0,0.0,1 );
  initial_guess.setVector( 0,pI);
  algorithm.initializeParameters(initial_guess);
  
  // OPTIONS
  algorithm.set( KKT_TOLERANCE, KKT_tol);
  algorithm.set( ABSOLUTE_TOLERANCE, 1e-4);
  algorithm.set( PRINTLEVEL,HIGH);
  algorithm.set( MAX_NUM_ITERATIONS, 1000 );
  algorithm.set (PRINT_COPYRIGHT, NO);
  // algorithm.set (PRINT_SCP_METHOD_PROFILE, YES);
    algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE);
  // algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN);
  algorithm.set(GLOBALIZATION_STRATEGY, GS_LINESEARCH ); 
  algorithm.set(LINESEARCH_TOLERANCE, 1e-2 ); 
  algorithm.set(INFEASIBLE_QP_HANDLING,IQH_RELAX_L2);
  algorithm.set(FEASIBILITY_CHECK,BT_TRUE);

  //SOLVING
  double clock1 = clock();
  algorithm.solve();
  double clock2 = clock();
  Vector solution;
  algorithm.getParameters(solution);

  //append the objective value to the parameter vector
  double obj_val=algorithm.getObjectiveValue();
  std::cout<<obj_val<<std::endl;
  solution.append(VectorspaceElement(1,&obj_val));


  // solution.print("optimal solution \n");
  solution.printToFile((path+"solution.dat").c_str(),"",PS_PLAIN);

  printf("\n computation time (ACADO) = %.16e \n", (clock2-clock1)/CLOCKS_PER_SEC);

  return 0;
}
//EOF

