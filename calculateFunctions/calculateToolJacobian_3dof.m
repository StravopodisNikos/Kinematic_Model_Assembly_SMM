function Jtool = calculateToolJacobian_3dof(J,qs)
% used for 3dof manipulator, when qs(spatial position vector of tool frame) is NOT a point of the last active twist

Jtool =   [   J(1,1) - J(6,1)*qs(2) + J(5,1)*qs(3), J(1,2) - J(6,2)*qs(2) + J(5,2)*qs(3), J(1,3) - J(6,3)*qs(2) + J(5,3)*qs(3);
              J(2,1) + J(6,1)*qs(1) - J(4,1)*qs(3), J(2,2) + J(6,2)*qs(1) - J(4,2)*qs(3), J(2,3) + J(6,3)*qs(1) - J(4,3)*qs(3);
              J(3,1) - J(5,1)*qs(1) + J(4,1)*qs(2), J(3,2) - J(5,2)*qs(1) + J(4,2)*qs(2), J(3,3) - J(5,3)*qs(1) + J(4,3)*qs(2)];
end