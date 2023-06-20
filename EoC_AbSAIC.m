function [rew,qp_table,qc_table,counter]=EoC_AbSAIC(scen,n,noa,ns,bits,inf_bits,best_rew,~,~,gamma,tau_k,ca,cs,pa,ps,ter,rew,temp_rew,counter,qc_table,qp_table,bsc_p,end_learn,update_tables)
                                                                 %(scen,n,noa,ns,bits,inf_bits,best_rew,worst_rew,goal_set,gamma,tau_k,ca,cs,pa,ps,ter,rew,temp_rew,counter,qc_table,qp_table,bsc_p_learn,end_learn_learn,update_tables)

%Action based State Aggregation for Information Compression 
%No learning or training happens here, we just use the dom_act_per_loc as
%the action policy :)
%load('agreggated_states_g16_infbits3','ag_states');
load('agreggated_states_n3_g9_infbits3','ag_states','dom_act_per_loc');
%% Numerical Simulator of Emergence of communication among rl agents under coordination environment
%Started: 19/12/2019
%Functions called : envir(ps,pa,n,noa) / envir_windy(ps,pa,n,noa,windy) , pdecide(ps,last_ps,cs,last_cs,scen,pa,temp_rew,rew_winner,tau), cdecide(ps,last_ps,ca,scen,temp_rew,rew_winner,tau)


%use new functions of cpolicy cosumized cupdate customized and ...

NE_table=0.001*ones(2,n^2,2^inf_bits,5);
NC_table=0.001*ones(2,n^2,2^inf_bits);

%% Setup
stuck_counter=0;


        %% Episode iteration
        for i=1:ns
            %Episode initialization
            disp(i)
            %waitbar(((b-1)/en)+i/(ns*b),h)
            temp_rew=0;
            rew_winner=[];
            counter(i)=1;

            % update tau, but above step 40'000 matlab will be unable to handle the
            % very big numbers so we don't go beyond...
            %tau=0.0001;

            if i<=40000
                tau=1/(1+i*tau_k);
            else
                tau=1/(1+40000*tau_k); 
            end

    
            %random initialization of position states and actions
            ps=randi(n*n-1,noa,1);
            
            %computed the corresponding aggregated ps
            ag_ps=s_aggregate(ps,inf_bits,noa,ag_states);

            pa=randi(5,noa,1);
    
            %random initialization of communication action ca, it depens on the scenario
            %selected
            switch scen
                case 1
                    ca=randi(2,noa,1);                             
                case 2
                    ca=ps;
                case 3
                    ca=randi(2,noa,bits)-ones(noa,bits);
                    ca_tilde=randi(2,noa,inf_bits)-ones(noa,inf_bits);
            end
    
        %episode run    
            while 3==3    
                        % why 3==3 ?
                        % while loop could not be conditioned on terminal state
                        % because after wev'e got to the terminal state still the 
                        % table updates should be done.
                        % Instead, at the end of each while loop it is checked if
                        % we've got to the terminal state or no
    
        
                
                ucb_counter=sum(counter(1:i));
                %SELECT COMM. ACTION AND UPDATE COMM. STATE
                %select communication action
                [ca_tilde] = cpolicy_customized_nbits_UCB(ag_ps,scen,tau,qc_table,NC_table,ucb_counter,ns,end_learn);
                                                                
                %Update UCBC counter
                NC_table(1,ps(1),bi2de(ca_tilde(1,:))+1)=NC_table(1,ag_ps(1),bi2de(ca_tilde(1,:))+1)+1;
                NC_table(2,ps(2),bi2de(ca_tilde(2,:))+1)=NC_table(2,ag_ps(2),bi2de(ca_tilde(2,:))+1)+1;
                
                %Encode the Communication Message
                ca = ca_tilde;
                
                %saving previouse communication state before it is updated
                last_cs=cs;
                %communication state update:
                %the previous ca of the other agent is the cs of the current
                %agent in case that no noise is added. Otherwise, noise
                %would change the signlas sent through the communication
                %channel.
                cs = ca;
                
                %cs=randi(2,2,1)-1;
                if counter(i)~=1  && update_tables==1 
                    %updating position table:
                    [qp_table] = pupdate_customized_nbits_2(ps,last_ps,cs,last_cs,scen,pa,temp_rew,rew_winner,tau,qp_table); 
                end
        
                %SELECT POSITION ACTION AND UPDATE POSITON STATE
                %select position action 
                [pa] = ppolicy_customized_nbits_UCB(ps,cs,scen,tau,qp_table,NE_table,ucb_counter,ns,end_learn);
        
                %Update UCBE counter
                for k=1:2
                    NE_table(k,ps(k),bi2de(transpose(squeeze(cs(k,1,:))))+1,pa(k))=NE_table(k,ps(k),bi2de(transpose(squeeze(cs(k,1,:))))+1,pa(k))+1;
                end
                
                %canceling action if the agent is in the terminal state
                for j=1:noa
                    if ps(j)==n^2
                        pa(j)=5;
                    end
                end
                                     
                %saving the previous position state before it is updated
                last_ps=ps;
                                       
                %environment, position state update
                [ps,~,ter]=envir(ps,pa,n,noa); 
                                                  %envir is a function which models the environment
                                                  %it tells if terminal state has
                                                  %been achieved, it tells if any
                                                  %error has occured
                                
                                          
                ag_ps=s_aggregate(ps,inf_bits,noa,ag_states);
                % Don't get stuck
                if length(find(last_ps==ps))==noa
                    stuck_counter=stuck_counter+1;
                    if stuck_counter==10
                        temp_rew=0;
                        if update_tables==1
% %                             [qc_table] = cupdate_customized_nbits_2(ps,last_ps,ca,scen,temp_rew,rew_winner,qc_table);
% %                             [qp_table] = pupdate_customized_nbits_2(ps,last_ps,cs,cs,scen,pa,temp_rew,rew_winner,tau,qp_table);
                            for k=1:noa
                                %the following line was commented out
                                %because that is enough to change qp_table
                                %to not get stuck any more
                                %qc_table(k,last_ps(k),bi2de(ca(k,:))+1)=median(qc_table(k,last_ps(k),:));
                                qp_table(k,last_ps(k),bi2de(transpose(squeeze(last_cs(k,1,:))))+1,pa(k))=median(qp_table(k,last_ps(k),bi2de(transpose(squeeze(last_cs(k,1,:))))+1,:));
                            end
                        end
                        break
                    end
                elseif length(find(last_ps==ps))~=noa
                    
                    stuck_counter=0;
                end


                % TERMINAL STATE CHECK                                  
                %check if we are in the terminal state and generate temp reward
                %which woul be used in table updating
                                                    %%%% make sure the ordering of
                                                    %%%% this module is
                                                    %%%% consistent
                                                    %%%% with its task, it should
                                                    %%%% update tables...
% %                 if ter>=1 
% %                                                   %calculating temp_reward
% %                                                   %this figure will be used to
% %                                                   %update q functions
% %                                           
% %                     temp_rew=1*best_rew^(ter-1)*gamma^counter(i);
% %                     rew_winner=[];
% %                     for ii=1:noa
% %                         if ps(ii)==n^2
% %                             rew_winner=[rew_winner,ii];
% %                         end  
% %                     end
% % 
% %                 end

                if ter>=2 
                                                  %calculating temp_reward
                                                  %this figure will be used to
                                                  %update q functions
                                          
                    temp_rew=best_rew;%*gamma^(counter(i));
                    rew_winner=[];
                    for ii=1:noa
                        if ps(ii)==n^2
                            rew_winner=[rew_winner,ii];
                        end  
                    end
                elseif ter==1
                    temp_rew=1;%*gamma^(counter(i));
                    rew_winner=[];
                    for ii=1:noa
                        if ps(ii)==n^2
                            rew_winner=[rew_winner,ii];
                        end  
                    end
                end
        
        
        
        
                %UPDATING TABLES:
                %updating communication table:
                %%%% when you have arrived to new episode, the last_ps
                %%%% would be the ending ps of the previous episode and
                %%%% gets unreal reward in this step this should be
                %%%% corrected
                if counter(i)==1
                %no update
                
                else 
                    if update_tables==1
                        [qc_table] = cupdate_customized_nbits(ps,last_ps,ca_tilde,scen,temp_rew,rew_winner,qc_table);
                    end
                end
                %counting the number of steps in the current episode
                counter(i)=counter(i)+1;
        
        
        
                %CHECKING WHILE LOOP CONDITION
                if ter >=1 
                                                  %in order to make sure that each loop is completed before termination
                                                  %while loop condition is always
                                                  %active but we check the
                                                  %condition at the end of each
                                                  %loop
                                          
                                                  %The reason is that we want to make sure that the q fucntions are updated even if we are in the terminal state
                    %updating position table:
                    if update_tables==1
                        [qp_table] = pupdate_customized_nbits_2(ps,last_ps,cs,cs,scen,pa,temp_rew,rew_winner,tau,qp_table);
                    end
                        term_hist(i)=ter;
                    break
                end
            end

        %episode summerize
            if temp_rew>1
                rew(i)=3*gamma^(counter(i)-1);
            else
                rew(i)=1*gamma^(counter(i)-1);
            end
% % % %             cumul_rew=cumul_rew+rew(i);
% % % %             saved_qc_8(i,:)=qc_table(1,n^2-1,:);
% % % %             saved_qc_6(i,:)=qc_table(1,n^2-n,:);
% % % %             saved_qp_8(i,:)=qp_table(1,n^2-1,1,:);
% % % %             saved_qp_6(i,:)=qp_table(1,n^2-n,1,:);
        end
        %save('qc3_table.mat','qc3_table')
        %save('qp3_table.mat','qp3_table')

end

%state aggregation based on V_o computed by centralized algorithm
%be very careful to import the correct ag_states
function ag_ps=s_aggregate(ps,inf_bits,noa,ag_states)
    ag_ps=zeros(2,1);
    for j=1:noa
        for k=1:2^inf_bits            
            if any(ag_states(k,:)==ps(j))
                ag_ps(j)=k;
            end
        end
    end
end
        