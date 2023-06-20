function [rew,qp_table,qc_table,counter,NE_table_emerged]=EoC_corrected_position_aggregated_UCB(scen,n,noa,ns,bits,inf_bits,best_rew,worst_rew,goal_set,gamma,tau_k,ca,cs,pa,ps,ter,rew,temp_rew,counter,qc_table,qp_table,bsc_p,end_learn,update_tables)


load('agreggated_states_g22_n8_infbits2_median','ag_states_median');
ag_states=ag_states_median;
%% Numerical Simulator of Emergence of communication among rl agents under coordination environment
%Started: 05/03/2018
%Functions called : envir(ps,pa,n,noa) / envir_windy(ps,pa,n,noa,windy) , pdecide(ps,last_ps,cs,last_cs,scen,pa,temp_rew,rew_winner,tau), cdecide(ps,last_ps,ca,scen,temp_rew,rew_winner,tau)


%use new functions of cpolicy cosumized cupdate customized and ...
%goal_set=[n*n];

NE_table=0.001*ones(2,n^2,2^bits,5);
%NC_table=0.001*ones(2,n^2,2^bits);
NE_table_emerged=0.001*ones(2,n^2,2^bits,5);

%% Setup
stuck_counter=0;
%inf_bits=length(de2bi(n*n-length(goal_set)));
%inf_bits=3;
cs=ones(noa,noa-1,inf_bits);  %only for position_based signalling scenario

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
                if bits>inf_bits

                        [ca] = de2bi(ag_ps-ones(2,1),inf_bits);
        % %                     gen=[1 0 0 1 0 1
        % %                          0 1 0 1 1 1
        % %                          0 0 1 0 1 1];
                        encData = encode(ca,bits,inf_bits,'cyclic/fmt');                                                

                elseif bits==inf_bits
                        [ca] = de2bi(ag_ps-ones(2,1),inf_bits);

                elseif bits<inf_bits
                        error('The number of bits selected is not enough to send the position information')
                        disp('The minimum number of bits which will not have any redundency is:')
                        length(de2bi(n*n-1,bits))
                end
                                             
        
                %saving previouse communication state before it is updated
                last_cs=cs;
                
                
                %communication state update:
                %the previous ca of the other agent is the cs of the current
                %agent in case that no noise is added. Otherwise, noise
                %would change the signlas sent through the communication
                %channel.
                if bits>inf_bits
                        %communication state update:
                        %the previous ca of the other agent is the cs of the current
                        %agent in case that no noise is added. Otherwise, noise
                        %would change the signlas sent through the communication
                        %channel.


                        for j=1:noa
                            cs_temp=encData;
                            cs_temp(j,:)=[];
                            cs_bc(j,:,:)=cs_temp;
                        end
                        %Sending cs_beforechannel to channel 
                        rec=bsc_ch(cs_bc,bsc_p);

                        for k=1:noa
                            cs(k,:) = decode(rec(k,:),bits,inf_bits,'cyclic/fmt');
                        end
                elseif bits==inf_bits

                        for j=1:noa
                            cs_temp=ca;
                            cs_temp(j,:)=[];
                            cs_bc(j,:,:)=cs_temp;
                        end
                        cs=bsc_ch(cs_bc,bsc_p);
%                         rec=bsc_ch(cs_bc,bsc_p);
% 
%                         for k=1:noa
%                             cs(k,:) = decode(rec(k,:),bits,inf_bits,'cyclic/fmt');
%                         end
                elseif bits<inf_bits
                        error('The number of bits selected is not enough to send the position information')
                        disp('The minimum number of bits which will not have any redundency is:')
                        length(de2bi(n*n-1,bits))
                end
                 
                

                if counter(i)~=1  && update_tables==1 
                    %updating position table:
                    [qp_table] = pupdate_customized_nbits_2(ps,last_ps,cs,last_cs,scen,pa,temp_rew,rew_winner,tau,qp_table); 
                
                    %CHECKING WHILE LOOP CONDITION
                    if ter >=1 
                                                      %in order to make sure that each loop is completed before termination
                                                      %while loop condition is always
                                                      %active but we check the
                                                      %condition at the end of each
                                                      %loop

                                                      %The reason is that we want to make sure that the q fucntions are updated even if we are in the terminal state
                        %updating position table:
                            term_hist(i)=ter;
                        break
                    end
                end
        
                %SELECT POSITION ACTION AND UPDATE POSITON STATE
                %select position action 
                [pa] = ppolicy_customized_nbits_UCB_bestrew(ps,cs,scen,tau,qp_table,NE_table,ucb_counter,ns,end_learn,best_rew);
                                                  %(ps,cs,scen,tau,qp_table,NE_table,ucb_counter,ns,end_learn)
                %Update UCBE counter
                for k=1:2
                    NE_table(k,ps(k),bi2de(transpose(squeeze(cs(k,1,:))))+1,pa(k))=NE_table(k,ps(k),bi2de(transpose(squeeze(cs(k,1,:))))+1,pa(k))+1;
                end
                
                %Update UCB_emerged counter
                if i>end_learn*ns                    
                    for k=1:2
                        NE_table_emerged(k,ps(k),bi2de(transpose(squeeze(cs(k,1,:))))+1,pa(k))=NE_table(k,ps(k),bi2de(transpose(squeeze(cs(k,1,:))))+1,pa(k))+1;
                    end
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
                [ps,~,ter]=envir_gc(ps,pa,n,noa,goal_set); 
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
                        if ps(ii)==goal_set
                            rew_winner=[rew_winner,ii];
                        end  
                    end
                elseif ter==1
                    temp_rew=1;%*gamma^(counter(i));
                    rew_winner=[];
                    for ii=1:noa
                        if ps(ii)==goal_set
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

                %counting the number of steps in the current episode
                counter(i)=counter(i)+1;
            end

        %episode summerize
            if temp_rew>1
                rew(i)=10*gamma^(counter(i)-1);
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
    ag_ps=ones(2,1);
    for j=1:noa
        for k=1:2^inf_bits            
            if any(ag_states(k,:)==ps(j))
                ag_ps(j)=k;
            end
        end
    end
end
        