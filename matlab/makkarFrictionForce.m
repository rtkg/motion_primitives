function f=makkarFrictionForce(dq,g)

f=g(1)*(tanh(g(2)*dq)-tanh(g(3)*dq))+g(4)*tanh(g(5)*dq)+g(6)*dq;
