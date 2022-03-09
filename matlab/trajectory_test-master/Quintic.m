function [p, v, a, t] = Quintic(t0, t1, dt, p0, p1, v0, v1, a0, a1)

    %{
        ##########   Quintic trajectory generation   ###########

        Inputs:
            t0  ==> time at initial point
            t1  ==> time at final point
            dt  ==> time step
            p0  ==> position of initial point
            p1  ==> position of final point
            v0  ==> velocity at initial point
            v1  ==> velocity at final point
            a0  ==> acceleration at initial point
            a1  ==> acceleration at final point

        Ouputs:
            p   ==> array of position points
            v   ==> array of velocity points
            a   ==> array of acceleration points
            t   ==> time array used to generate trajectory
        
    %}


    % time matrix
    A = [   
            1, t0, t0^2, t0^3, t0^4, t0^5;
            1, t1, t1^2, t1^3, t1^4, t1^5;
            0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
            0, 1, 2*t1, 3*t1^2, 4*t1^3, 5*t1^4;
            0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
            0, 0, 2, 6*t1, 12*t1^2, 20*t1^3
    ];

    % trajectory constraints
    b = [   
            p0;
            p1;
            v0;
            v1;
            a0;
            a1
    ];

    % solving the six coefficients for the fifth order polynomial
    % A*x = b  ==>   x = A^(-1)*b
    x = A\b;

    % initialize index for arrays
    idx = 1;
    
    t = (t0:dt:t1)';    % time array
    N = size(t);        % number of samples
    
    % pre-allocate size for arrays
    p_ = zeros(N);
    v_ = zeros(N);
    a_ = zeros(N);

    for t_=t0:dt:t1

        p_(idx) = x(1) + x(2)*t_ + x(3)*t_^2 + x(4)*t_^3 + x(5)*t_^4 + x(6)*t_^5;   % position
        v_(idx) = x(2) + 2*x(3)*t_ + 3*x(4)*t_^2 + 4*x(5)*t_^3 + 5*x(6)*t_^4;       % velocity
        a_(idx) = 2*x(3) + 6*x(4)*t_ + 12*x(5)*t_^2 + 20*x(6)*t_^3;                 % acceleration

        idx = idx + 1;  % increment index
    end
    
    % write to output variables
    p = p_;
    v = v_;
    a = a_;
end