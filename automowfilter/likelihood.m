function e = likelihood( Mu, P_x, C )
[V,D] = eig(P_x);
theta = 2*pi*[0:0.001:1];
for jj = 1:length(C),
    for ii = 1:length(theta),
        e(:,ii) = Mu + ...
            C(jj) * chol(P_x,'lower')*[cos(theta(ii));sin(theta(ii))];
    end
    hold on
end
end

