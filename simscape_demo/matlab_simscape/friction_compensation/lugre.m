function vel = lugre(v)
    sigma_0 = 1e5;
    sigma_1  = sqrt(1e5);
    sigma_2  = 0.4;
    Fc = 1;
    Fs = 1.5;
    vs = 0.001;
    vel = lugref_ss(v,Fc,Fs,vs,sigma_2)
end
