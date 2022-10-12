

classdef Game


    methods (Static)

        % return all the wining layouts in a 3*3*8 logical matrix
        function mat = winMatrix()
            mat = zeros(0);
            for i = 1:3
                for j = 1:3
                    mat(i,j,i) = 1;     % three horizontal
                    mat(j,i,i+3) = 1;   % three vertical
                end
                mat(i,i,7) = 1;         % two cross
                mat(i,4-i,8) = 1;
            end
            mat = logical(mat);
        end
        



        % given a layour of the board in form of 3*3 logical matrix
        % return    zeros(0) when already won
        %           3*3 matrix of missing piece if there is a way to win
        %           [0] if no way to win
        function miss_mat = getMissMatrix(current_layout, empty_layout)
            win_mat = Game.winMatrix();
            min_miss_num = Inf;
            min_miss_layout = ones(3);
            for i = 1:size(win_mat,3)
                win_layout = win_mat(:,:,i);
                % do & with win and reverse current layout 
                % to get the pieces needed
                miss_layout = win_layout & ~current_layout;
                % calculate how many pieces are missing
                miss_num = sum(sum(miss_layout));
                if (miss_num == 0)
                    warning("Already won.");
                    miss_mat = zeros(0);
                    return
                end

                if (miss_num == 1)
                    % check if miss is still empty
                    still_empty_layout = miss_layout & empty_layout;
                    still_empty = any(any(still_empty_layout));
                    if still_empty
                        min_miss_num = miss_num;
                        min_miss_layout = miss_layout;
                    end
                end
            end

            if (min_miss_num == 1)
                % can win, return the missing piece as 3*3 logical matrix
                miss_mat = logical(min_miss_layout);
            else
                % no way to win, return [0]
                miss_mat = zeros(1);
            end
%             disp("Miss Matrix");
%             disp(miss_mat);
        end



        % given computer and human layout, each 3*3 matrix
        % return    [] board already filled
        %           [0] human already won
        %           [1] computer already won
        %           [row, col] of the computer move
        function move_pos = computerMove(computer_layout, human_layout)
            if any(any(computer_layout & human_layout))     % layout overlap 
                error(["Pieces over lap", computer_layout & human_layout]);
            end

            empty_layout = ~ (computer_layout | human_layout);
            if ~ any(any(empty_layout))     % empty layout is empty
                warning("Board already filled");
                move_pos = zeros(0);
                return
            end
%             disp("Computer Miss");
            computer_miss = Game.getMissMatrix(computer_layout, empty_layout);
            computer_miss_size = size(computer_miss, 1);
            if computer_miss_size == 0
                % already won
                move_pos = [1];
                return
            elseif computer_miss_size == 3
                % wining move, return pos
                [row, col] = find(computer_miss == 1);
                move_pos = [row col];

            else
                % no wining move
                % prevent human from winning
                disp("Human Miss");
                human_miss = Game.getMissMatrix(human_layout, empty_layout);
                human_miss_size = size(human_miss, 1);
                if human_miss_size == 0
                    % human already won
                    move_pos = [0];
                    return
                elseif human_miss_size == 3
                    % human going to win, return pos 
                    [row, col] = find(human_miss == 1);
                    move_pos = [row col];
                else
                    % no one can win
                    % go random / center
                    disp("No good move");
                    center_layout = [0 0 0; 0 1 0; 0 0 0];
                    if any(any(empty_layout & center_layout))
                        % center is empty
                        disp("go center");
                        move_pos = [2 2];
                    else
                        disp("go random");
                        % index where board is empty
                        empty_index = find(empty_layout == 1);
                        rand_index = randi([1,length(empty_index)]);
                        chosen_index = empty_index(rand_index);
                        % get the row, col for the random index
                        move_pos = [mod(chosen_index-1, 3)+1, floor((chosen_index-1)/3)+1];
                    end
                end
                
            end
            disp("Computer move:");
            disp(move_pos);

        end


        % given computer and human layout
        % return    1 for computer win
        %           -1 for human win
        %           0 for no win
        function win = checkWin(computer_layout, human_layout)
            disp("Checking Win");
            empty_layout = ~ (computer_layout | human_layout);

            computer_miss = Game.getMissMatrix(computer_layout, empty_layout);
            if size(computer_miss,1) == 0
                disp("Computer Win");
                win = 1;
                return;
            end

            human_miss = Game.getMissMatrix(human_layout, empty_layout);
            if size(human_miss,1) == 0
                disp("Human Win");
                win = -1;
                return;
            end

            win = 0;
        end



        % Function for debuging
        % given row, col you choose to move 
        %   and the current layout in 3*3*2 [computer_lay; hum_layout]
        % register your move, and then computer move
        % display board each time
        % return
        function ret_layout_mat = humanMove(row,col,layout_mat)
            computer_layout = layout_mat(:,:,1);
            human_layout = layout_mat(:,:,2);
            ret_layout_mat = layout_mat;
            if human_layout(row, col) == 1 || computer_layout(row, col) == 1
                error("Place already taken. Row %d Col %d", row, col);
            end
            % place human move
            ret_layout_mat(row,col,2) = 1;

            % display board
            disp_mat = ["-"];
            for i = 1:3
                for j = 1:3
                    % computer layout on board
                    if ret_layout_mat(i,j,1) == 1
                        disp_mat(i,j) = "X";
                    elseif ret_layout_mat(i,j,2) == 1
                        disp_mat(i,j) = "O";
                    else
                        disp_mat(i,j) = "-";
                    end
                end
            end
            disp(disp_mat);
            
            % if someone win, stop the game
            if Game.checkWin(ret_layout_mat(:,:,1), ret_layout_mat(:,:,2))
                return
            end

            computer_move = Game.computerMove(ret_layout_mat(:,:,1), ret_layout_mat(:,:,2));
            % not normal move
            if length(computer_move) ~= 2
                warning("Game already finished");
                if computer_move == 1
                    warning("Computer Win");
                elseif computer_move == 0
                    warning("Human Win");
                else
                    warning("Board Filled");
                end
                ret_layout_mat = zeros(0);
                return
            end
            ret_layout_mat(computer_move(1), computer_move(2), 1) = 1;

            disp_mat(computer_move(1), computer_move(2)) = "X";
            disp(disp_mat);


            if Game.checkWin(ret_layout_mat(:,:,1), ret_layout_mat(:,:,2))
                return
            end


        end













    end



end





