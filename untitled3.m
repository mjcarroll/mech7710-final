clear classes

x_hat_i = [1, 1, 0, 1, 1, 1];

test = LawnmowerModel(x_hat_i,eye(6),eye(6),eye(2),eye(1))
[x_hat, P]=test.TimeUpdate([0.99;1],0.01)