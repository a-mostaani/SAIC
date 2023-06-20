%% Parallel simulator
clear
%close all
clc


% Numerical Simulator of Emergence of communication among rl agents under coordination environment
%Started: 05/03/2018
%Functions called : envir(ps,pa,n,noa) / envir_windy(ps,pa,n,noa,windy) , pdecide(ps,last_ps,cs,last_cs,scen,pa,temp_rew,rew_winner,tau), cdecide(ps,last_ps,ca,scen,temp_rew,rew_winner,tau)


%% Control variables
scen=3;             
                    %communication scenario
n=4;                
                    %size of gridworld
noa=2;              
                    %number of agents
bits=2;
%ns=fix(55000*(2^(bits-7)));
ns=10000;
ns_exec=10000;
ep=1;              %error points

                    %number of simulations in each epoch
                    %based on experience: 10k would be enough up to n=3
                    % 100K for n=4
                    % 200K for n=5
avg_len=ns*0.1;                    

en=1;
                    %number of epochs

err_trig=0.3;
best_rew=3;
worst_rew=1;
goal_set=[n*n];

bsc_p_learn=0.15; 
bsc_p_exec=0.15;

end_learn_learn=0.85;
end_learn_exec=0.05;
%windy_envir        
                    %if you need a windy environment, you can choos windy_envir
                    %function instead of envir
update_tables=1;
%variables that can be modified in cdedcie():
%alpha=0.2; 
%sweep=0; %if sweep is one, we update the whole q_table at the time that the
         % function cdecide is called
         %if sweep is off, then only update the q_table for the current
         %state of the agents
gamma=0.9;


                    
tau_k=0.005;        
                    %the constant value based on which tau will be updated in each
                    %new episode
%term_hist=zeros(ns,1);

%h=waitbar(0,'Processing');


%% Initialization
epoch_rew=zeros(ns,en);
learning_rew=zeros(ns,en);
epoch_rew_de=zeros(ns_exec,en,ep);
learning_rew_de=zeros(ns,en,ep);
epoch_counter_learn=zeros(ns,en);
epoch_counter_exec=zeros(ns,en);
epoch_NC_table=zeros(ep,2,n^2,2^bits);
epoch_NE_table=zeros(ep,2,n^2,2^bits,5);



%disp(b)
%wind=zeros(1,2);
                    %if environment is windy, it can get 0 or 1 in x or y
                    %direction
%wind_loc           
                    %please note that if you want to change the setting
                    %related to the wind_loc you should do it in the
                    %initialization part of envir_windy


ca=zeros(noa,bits);      
                    %communication action of each of agents (each row)
                    %it can be specified based on communcation policy, there are
                    %two policies studied in this paper:
                    %1-sending the current position
                    %2-learning how to communicate using one bit of data

%policy             
                    %if you want to change the policy you should change it
                    %in cdecide or pdecide functions
cs=ones(noa,noa-1,bits);      
                    %communication state of each of agents (each row)
                    %This is equal to communication action of the other agents in
                    %the previous step (each column and its depth)
                    %since each agent might send more than one
                    %bits, the matrix contains depth, the matrix is
                    %as deep as the number bits sent by each agent
pa=randi(5,noa,1);      
                    %position action of each of agents (each row)
                    %done based on RL 
ps=randi(n*n-1,noa,1);
                    %position state of each of agents (each row)
                    %based on pa, environment will do the calculations to
                    %determine the nex position state
                    %at initialization, this value is determinde randomly
                    %but can't be the "terminal state"
ter=0;
                    %indicates if the terminal state has been achieved or
                    %not and if it has been achieved how many agents has
                    %been into it. e.g. ter=0 : not achived
                                       %ter=1 : achived, one agent on it
                                       %ter=2 : achived, two agent on it
                                       %ter=n : achived, n agent on it
rew=zeros(ns,1);    
                    %general rward for each episode simulation
temp_rew=0;
                    %this value is used inside while loop to be transferred
                    %to cdecide and pdecide when a reward has been achieved
                    %this would let the q function for that state action
                    %being updated

counter=zeros(ns,1);
                    %number of steps taken in each episode simulaton


%last_ps=zeros(noa,1); %last position state of each agent
%last_cs=zeros(noa,1); %last communication state of each agent

%cumul_rew=0;

qc_table=0.02*ones(noa,n^2,2^bits);
qc=0.02*ones(noa,n^2,2^bits);
                    %qc3_table(n^2,:)=[2,2];
                    %the q table of communication actions for two
                    %agetns, qc_table(i,:,:) is qc table of ith agent
                    %based on the number of bits that can be sent
                    %by each agent, there are 2^b com actions that
                    %can be done by it.

switch scen
   case 1
       qp_table=0.02*ones(noa,n^2,2,5);
   case 2
       qp_table=0.02*ones(noa, n^2,n^2,5);
   case 3
       qp_table=0.02*ones(noa,n^2,2^bits,5);
       qp=0.02*ones(noa,n^2,2^bits,5);
end
                    %the q table of position actions for two agents
                    %qp_table(i,:,:,:) is qp table of ith agent


%load('qc3_table.mat','qc3_table')
%load('qp3_table.mat','qp3_table')

%saved_qc_8=zeros(ns,2^bits);
                        %saving the 8th row of qc table throughout time to
                        %to see how it evolves
%saved_qc_6=zeros(ns,2^bits);
%saved_qp_8=zeros(ns,5);
                        %saving the 8th row of qp table throughout time to
                        %to see how it evolves
%saved_qp_6=zeros(ns,5);

%% Main function


bsc_p_learn_main=bsc_p_learn; 
bsc_p_exec_main=bsc_p_exec;
tic

for k=1:ep
    bsc_p_learn=bsc_p_learn_main*k/ep;
    bsc_p_exec= bsc_p_exec_main *k/ep;
    
    for i=1:en
        %learning phase
        [learning_rew(:,i),qp,qc,epoch_counter_learn(:,i)]=EoC_corrected_UCB(scen,n,noa,ns,bits,best_rew,worst_rew,gamma,tau_k,ca,cs,pa,ps,ter,rew,temp_rew,counter,qc_table,qp_table,bsc_p_learn,end_learn_learn,update_tables);

        %execution phase
        [epoch_rew(:,i),epoch_NC_table(i,:,:,:),epoch_NE_table(i,:,:,:,:)]=Hist_EoC_corrected_UCB(scen,n,noa,ns_exec,bits,best_rew,worst_rew,gamma,tau_k,ca,cs,pa,ps,ter,rew,temp_rew,counter,qc,qp,bsc_p_exec,end_learn_exec,0);

    end
    %epoch_rew_de(:,:,k)=epoch_rew;
    learning_rew_de(:,:,k)=learning_rew;
end

toc






%% Visualization
%mean_rew_exec=zeros(1,ep);
mean_rew_learn=zeros(1,ep);
for i=1:ep
%mean_rew_exec(i)=mean(mean(epoch_rew_de(fix(ns*end_learn_exec+1):end,:,i)));
mean_rew_learn(i)=mean(mean(learning_rew_de(fix(ns*end_learn_learn+1):end,:,i)));
end


y2=zeros(ns,ep);
for i=1:ep
    epi=i;
    avg_len=100;
    avg_rew=zeros(ns-avg_len,1);
    for j=1:ns-avg_len
        avg_rew(j)=mean(mean(learning_rew_de(j:j+avg_len,:,epi),2));
        
    end


    f=3;
    figure(f)
    hold on
    %plot(avg_rew)
    y2(1:length(movmean(avg_rew,0.05*ns)),i)=movmean(avg_rew,0.05*ns);
    plot(y2)
    xlabel("Steps")
    ylabel("(Moving average applied on) rewards")
    title("Reward improvement through time - No Delay Communication")
    grid minor
end


%calculating the entropy of Environment State
epoch_NC_table=epoch_NC_table(:,:,1:n^2-1,:); %this is to remove the action counts on terminal state

%epoch_NC_table=fix(mean(epoch_NC_table,1)); %taking mean over all of the epochs
H_se_en=zeros(en,2);
p_se_en=zeros(n^2,en,2);
H_ac_en=zeros()
for j=1:en
        total_count_1=sum(sum(sum(epoch_NC_table(j,1,:,:)))); %calculating total number of times all action-states are visited
        epoch_NC_table_1=epoch_NC_table/total_count_1;    %normalizing the counted values of state-actions


        %calculating the entropy of state probability distribution
        p_se_1=zeros(n^2,1);
        H_se_1=0;
        for i=1:n^2-1
            p_se_1(i)=sum(epoch_NC_table_1(j,1,i,:));
            H_se_1=H_se_1+p_se_1(i)*log(p_se_1(i));
        end
        H_se_1=-H_se_1;

        %calculating the entropy of actions probability distribution
        p_ac_1=zeros(2^bits,1);
        H_ac_1=0;
        for i=1:2^bits
            p_ac_1(i)=sum(epoch_NC_table_1(j,1,:,i));
            H_ac_1=H_ac_1+p_ac_1(i)*log(p_ac_1(i));
        end
        H_ac_1=-H_ac_1;

        %calculating the entropy of state-actions probability distribution
        p_seac_1=epoch_NC_table_1(j,1,:,:);
        H_seac_1=-sum(sum(epoch_NC_table_1(j,1,:,:).*log(epoch_NC_table_1(j,1,:,:))));

        minf_seac_1=H_se_1+H_ac_1-H_seac_1


        %calculating entropy for 2nd agent
        total_count_2=sum(sum(sum(epoch_NC_table(j,2,:,:))));
        epoch_NC_table_2=epoch_NC_table/total_count_2;

        %calculating the entropy of state probability distribution for 2nd agent
        p_se_2=zeros(n^2,1);
        H_se_2=0;
        for i=1:n^2-1
            p_se_2(i)=sum(epoch_NC_table_2(j,2,i,:));
            H_se_2=H_se_2+p_se_2(i)*log(p_se_2(i));
        end
        H_se_2=-H_se_2;

        %calculating the entropy of actions probability distribution
        p_ac_2=zeros(2^bits,1);
        H_ac_2=0;
        for i=1:2^bits
            p_ac_2(i)=sum(epoch_NC_table_2(j,2,:,i));
            H_ac_2=H_ac_2+p_ac_2(i)*log(p_ac_2(i));
        end
        H_ac_2=-H_ac_2;

        %calculating the entropy of state-actions probability distribution
        p_seac_2=epoch_NC_table_2(1,2,:,:);
        H_seac_2=-sum(sum(epoch_NC_table_2(1,2,:,:).*log(epoch_NC_table_2(1,2,:,:))));

        minf_seac_2=H_se_2+H_ac_2-H_seac_2

end

p_se_1_grid=zeros(n,n);
for i=1:n
    for j=1:n
        p_se_1_grid(i,j)=p_se_1((i-1)*n+j);
    end
end

figure
imagesc(p_se_1_grid);


std_mat=zeros(ns,ep);
for i=1:ep
    std_mat(:,i)=std(learning_rew_de(:,:,i),0,2);
end
std_mat=movmean(std_mat,0.01*ns);

% for i=1:ep
%     hold on
%     errorbar(1:2000:20000,y2(1:2000:end,i),std_mat(1:2000:end,i))
% end
%  
% save('qc_table_errorcorrection.mat','qc')
% save('qp_table_errorcorrection.mat','qp')