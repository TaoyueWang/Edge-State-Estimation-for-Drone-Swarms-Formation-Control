close all; 
clear;
clc;
%% load data
load("data.mat");
%% plot formation control system without noise and with noise
% parameter initialization
K = 50000;  % total number of iterations
mu = [0 0];
% estimate the states for each iteration
z_record = control_without_noise(N,K,z,L,dt);
z_record_noise = control_with_noise(N,K,z,L,dt,mu,R);
% calculate the estimation error for each iteration
error = calculate_error(z_record,z_star);
error_noise = calculate_error(z_record_noise,z_star);
% plot the convergence in terms of error
figure;
semilogy(1:K,error,1:K,error_noise,'LineWidth',2);
legend('No noise','With noise');
title("Error convergence plot",'FontSize',17,'FontWeight','bold')
xlabel("Number of iterations",'FontSize',13,'FontWeight','bold')
ylabel("Error",'FontSize',13,'FontWeight','bold')
% plot the trajectories of convergence
plot_trajectory(z_record);
title("Trajectory of convergence (no noise,T=10)",'FontSize',17,'FontWeight','bold')
plot_trajectory(z_record_noise);
title("Trajectory of convergence (with noise,T=10)",'FontSize',17,'FontWeight','bold')
%% plot MLE results
% estimate the states for each iteration
z_record_mle_T10 = control_with_mle(N,K,z,L,dt,mu,R,10);
z_record_mle_T100 = control_with_mle(N,K,z,L,dt,mu,R,100);
% calculate the estimation error for each iteration
error_mle_T10 = calculate_error(z_record_mle_T10,z_star);
error_mle_T100 = calculate_error(z_record_mle_T100,z_star);
% plot the convergence in terms of error
figure;
semilogy(1:K,error,1:K,error_noise,1:K,error_mle_T10,1:K,error_mle_T100,'LineWidth',2);
legend('No noise','With noise','MLE (T=10)','MLE (T=100)');
title("Error convergence plot",'FontSize',17,'FontWeight','bold')
xlabel("Number of iterations",'FontSize',13,'FontWeight','bold')
ylabel("Error",'FontSize',13,'FontWeight','bold')
% plot the trajectories of convergence
plot_trajectory(z_record_mle_T10);
title("Trajectory of convergence (MLE,T=10)",'FontSize',17,'FontWeight','bold')
plot_trajectory(z_record_mle_T100);
title("Trajectory of convergence (MLE,T=100)",'FontSize',17,'FontWeight','bold')
%% plot MMSE results
%% T=10, change the variance
% parameter initialization
var_prior_5 = 1e-5;  % for MMSE estimator only: the variance of the prior pdf for the relative position
var_prior_4 = 1e-4;
var_prior_6 = 1e-6;
% estimate the states for each iteration
z_record_mmse_4 = control_with_mmse(N,K,z,L,dt,mu,R,T,var_prior_4);
z_record_mmse_5 = control_with_mmse(N,K,z,L,dt,mu,R,T,var_prior_5);
z_record_mmse_6 = control_with_mmse(N,K,z,L,dt,mu,R,T,var_prior_6);
% calculate the estimation error for each iteration
error_mmse_4 = calculate_error(z_record_mmse_4,z_star);
error_mmse_5 = calculate_error(z_record_mmse_5,z_star);
error_mmse_6 = calculate_error(z_record_mmse_6,z_star);
% plot the convergence in terms of error
figure;
semilogy(1:K,error,1:K,error_noise,1:K,error_mle_T10,1:K,error_mmse_4,1:K,error_mmse_5,1:K,error_mmse_6,'LineWidth',2);
legend('No noise','With noise','MLE','MMSE (\sigma_{prior}^2 = 1E - 4)','MMSE (\sigma_{prior}^2 = 1E - 5)','MMSE (\sigma_{prior}^2 = 1E - 6)');
title("Error convergence plot (T=10)",'FontSize',17,'FontWeight','bold')
xlabel("Number of iterations",'FontSize',13,'FontWeight','bold')
ylabel("Error",'FontSize',13,'FontWeight','bold')
% plot the trajectories of convergence
plot_trajectory(z_record_mmse_4);
title("Trajectory of convergence (MMSE,\sigma_{prior}^2 = 1E - 4)",'FontSize',17,'FontWeight','bold')
plot_trajectory(z_record_mmse_5);
title("Trajectory of convergence (MMSE,\sigma_{prior}^2 = 1E - 5)",'FontSize',17,'FontWeight','bold')
plot_trajectory(z_record_mmse_6);
title("Trajectory of convergence (MMSE,\sigma_{prior}^2 = 1E - 6)",'FontSize',17,'FontWeight','bold')
%% T=10 & 100
% parameter initialization
var_prior = 1e-5;  % for MMSE estimator only: the variance of the prior pdf for the relative position
% estimate the states for each iteration
z_record_mmse_10 = control_with_mmse(N,K,z,L,dt,mu,R,10,var_prior);
z_record_mmse_100 = control_with_mmse(N,K,z,L,dt,mu,R,100,var_prior);
% calculate the estimation error for each iteration
error_mmse_10 = calculate_error(z_record_mmse_10,z_star);
error_mmse_100 = calculate_error(z_record_mmse_100,z_star);
% plot the convergence in terms of error
figure;
semilogy(1:K,error,1:K,error_noise,1:K,error_mmse_10,1:K,error_mmse_100,'LineWidth',2);
legend('No noise','With noise','MMSE (T=10)','MMSE (T=100)');
title("Error convergence plot (\sigma_{prior}^2 = 1E - 5)",'FontSize',17,'FontWeight','bold')
xlabel("Number of iterations",'FontSize',13,'FontWeight','bold')
ylabel("Error",'FontSize',13,'FontWeight','bold')
% plot the trajectories of convergence
plot_trajectory(z_record_mmse_10);
title("Trajectory of convergence (MMSE,T=10)",'FontSize',17,'FontWeight','bold')
plot_trajectory(z_record_mmse_100);
title("Trajectory of convergence (MMSE,T=100)",'FontSize',17,'FontWeight','bold')
%% plot Kalman results
% estimate the states for each iteration
z_record_kalman_T10 = control_with_kalman(N,K,z,L,dt,mu,R,10);
z_record_kalman_T100 = control_with_kalman(N,K,z,L,dt,mu,R,100);
% calculate the estimation error for each iteration
error_kalman_T10 = calculate_error(z_record_kalman_T10,z_star);
error_kalman_T100 = calculate_error(z_record_kalman_T100,z_star);
% plot the convergence in terms of error
figure;
semilogy(1:K,error,1:K,error_noise,1:K,error_kalman_T10,1:K,error_kalman_T100,'LineWidth',2);
legend('No noise','With noise','Kalman (T=10)','Kalman (T=100)');
title("Error convergence plot",'FontSize',17,'FontWeight','bold')
xlabel("Number of iterations",'FontSize',13,'FontWeight','bold')
ylabel("Error",'FontSize',13,'FontWeight','bold')
% plot the trajectories of convergence
plot_trajectory(z_record_kalman_T10);
title("Trajectory of convergence (Kalman,T=10)",'FontSize',17,'FontWeight','bold')
plot_trajectory(z_record_kalman_T100);
title("Trajectory of convergence (Kalman,T=100)",'FontSize',17,'FontWeight','bold')
%% comparison of three estimators(T=10)
% plot the convergence in terms of error
figure;
semilogy(1:K,error,1:K,error_noise,1:K,error_mle_T10,1:K,error_mmse_10,1:K,error_kalman_T10,'LineWidth',2);
legend('No noise','With noise','MLE','MMSE','Kalman');
title("Error convergence plot (T=10)",'FontSize',17,'FontWeight','bold')
xlabel("Number of iterations",'FontSize',13,'FontWeight','bold')
ylabel("Error",'FontSize',13,'FontWeight','bold')