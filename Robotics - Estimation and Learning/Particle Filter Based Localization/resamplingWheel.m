function [index] = resamplingWheel(w)
	N = numel(w);
	scale_w = cumsum(w / sum(w));
	index = [];
	for i = 1:N
		pick = rand();
		smaller_id = find(scale_w > pick);
		index = [index, smaller_id(1)];
	end
end

