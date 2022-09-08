% Calculation of inverse dynamics.
% using the recursive Newton-Euler algorithm, evaluated in link coordinates.
% --------------------Nomenclature----------------------------------
% q = configuration ; qd = velocity ; qdd = acceleration ; length = length
% of the link ; int = distance between center of mass and joint ; pos_int =
% position of center of mass initially ; pos_com = position of center of
% mass after displacement ; zar = z-axis after rotation ; omg = angular 
% velocity ; omgd = angular acceleration ; ldd = linear
% acceleration; ldc = linear acceleration of center of mass ; omgt = joint
% velocity of the translational matrix ; omgtd = joint acceleration of the
% translational matrix ; lddt = joint velocity of transformation matrix ;
% ldct = of center of mass ; force ; p = moment ; I = inertia ; m = mass ;
% forcet = force of transformation matrix ; pt = momentum of t.m ; taut =
% of transf. matrix ; tau = Torque
clear all
clc

%---------------Forward rescursion---------------------------------

input(sprintf("Note: The length must be 1 unit higher than the DoF"));
length = input('Length: '); % need to enter input in command window

for i = 1 : length-1
    q(i) = input('q : ');
    q(i) = deg2rad(q(i));
end

for i = 1 : length-1
    qd(i) = input('enter qdot : ');
    qdd(i) = input('enter qdoubledot : ');
end
grav_accn = [0;-9.81;0] % gravity = 9.81 m/s2

for i = 1 : length-1
    l(i) = input('enter length: ')  
end

int = [0 0 0 0]; % distance ;in this case its in the origin of the reference frame

for i = 2 : length
    int(i) = l(i-1)/2; % distance between origin and center of mass
end

pos_int = [[0 0 0]',[0 0 0]', [0 0 0]', [0 0 0]'];
pos_com = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];

for i = 1 : length-1
    pos_int(1:3,i) = [l(i)*cos(q(i)) ; l(i)*sin(q(i)); 0];
end
for i = 1 : length-1
    pos_com(1:3,i+1) = [int(i+1)*cos(q(i)) ; int(i+1)*sin(q(i)); 0];
end

zar = [0 0 1]'; % z-axis after rotation

omg = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]']; % we have initialized the dimensions of the matrix
for i = 2 : length
    omg(1:3,i) = [0 ; 0 ; omg(3,i-1)+(zar(3,1)*qd(i-1))]; % angular velocity is being calculated here
end
omgd = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
for i = 2 : length
    omgd(1:3,i) = [0 ; 0 ; omgd(3,i-1)+(omg(3,i)*zar(3,1))*qd(i-1)+(zar(3,1)*qdd(i-1))]; % angular acceleration is calculated
end
ldd = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
for i = 2 : length
    ldd(1:3,i) = ldd(1:3,i-1)+((omgd(3,i) .* pos_int(1:3,i)) + (omg(3,i) .* (omg(3,i) .* pos_int(1:3,i))));
end
ldc= [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
for i = 2 : length
    ldc(1:3,i) = ldd(1:3,i-1)+((omgd(3,i) .* pos_com(1:3,i-1)) + (omg(3,i) .* (omg(3,i) .* pos_com(1:3,i-1))));
end
 % Note : At the end of each iteration, a matrix has been initialized ,
 % then the angular acceleration and the other parameters of the
 % transformation matrix has been calculated .All the paremeters have been
 % explained in the "Nomenclature" section.
   
       
omgt = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]']; % note : every matrix has been initialized first
for i = 2 : length
    omgt(1:3,i) = [0 ; 0 ; omgt(3,i-1)];
end

omgtd = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
for i = 2 : length
    omgtd(1:3,i) = [0 ; 0 ; omgtd(3,i-1)];
end

lddt = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
for i = 2 : length
    lddt(1:3,i) = (lddt(1:3,i-1) + qdd(2) .* zar) + ((2 .* qd(2) .* omgt(3,i) .* zar)) + (omgtd(3,i) .* pos_int(1:3,i-1)) + omgt(3,i) .* (omgt(3,i) .* pos_int(1:3,i-1));
end
ldct = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
for i = 2 : length
    ldct(1:3,i) = lddt(1:3,i-1)+((omgtd(3,i) .* pos_com(1:3,i-1)) + (omgt(3,i) .* (omgt(3,i) .* pos_com(1:3,i-1))));
end

%------------------------Backward Recursion-------------------
force = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
p = [[0 0 0]',[0 0 0]',[0 0 0]', [0 0 0]'];
tau = [0 0 0]';

for i = 1 : length - 1
    m(i) = input('mass: ');
end

for i = 1 : length - 1
    I(i) = input('Inertia: ');
end

for i = length-1 : -1 : 1
    force(1:3,i) = force(:,i+1) + (m(i) .* ldc(:,i+1));
end

for i = length-1 : -1 : 1
    p(1:3,i) = (-force(:,i) .* (pos_int(1:3,i) + pos_com(1:3,i))) + p(1:3,i+1) + (force(1:3,i+1) .* pos_com(1:3,i)) + (I(i) .* omgd(1:3,i+1)) + (omg(1:3,i+1) .* (I(i) .* omg(1:3,i+1)));
end

for i = length-1 : -1 : 1
    force(1:3,i) = force(:,i+1) + (m(i) .* ldc(:,i+1));
end

for i = length-1 : -1 : 1
    p(1:3,i) = (-force(:,i) .* (pos_int(1:3,i) + pos_com(1:3,i))) + p(1:3,i+1) + (force(1:3,i+1) .* pos_com(1:3,i)) + (I(i) .* omgd(1:3,i+1)) + (omg(1:3,i+1) .* (I(i) .* omg(1:3,i+1)));
end
for i = length : -1 : 1
    tau(:,i) = p(:,i) .* zar;
end
   
for i = length-1 : -1 : 1
    forcet(1:3,i) = force(:,i+1) + (m(i) .* ldct(:,i+1));% computing the force of translational joint
end

for i = length-1 : -1 : 1
    pt(1:3,i) = (-force(:,i) .* (pos_int(1:3,i) + pos_com(1:3,i))) + p(1:3,i+1) + (force(1:3,i+1) .* pos_com(1:3,i)) + (I(i) .* omgtd(1:3,i+1)) + (omgt(1:3,i+1) .* (I(i) .* omgt(1:3,i+1)));
end
for i = length : -1 : 1
    taut(:,i) = force(:,i) .* zar;
end