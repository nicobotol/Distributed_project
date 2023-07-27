function bb = bounding_box(xlim, ylim)
    bb = rectangle('Position',[xlim(1) ylim(1) xlim(2)-xlim(1) ylim(2)-ylim(1)], 'EdgeColor', 'b');
end