function im = flipImage(im, isUpsideDown, isMirrored)
    if isUpsideDown
        im = im(end:-1:1,:);
    end
    if isMirrored
        im = im(:,end:-1:1);
    end
end % function
