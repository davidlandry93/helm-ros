
(defvar ros-process-mode-hook nil)

(defvar ros-process-mode-map
  (let ((map (make-keymap)))
    (define-key map (kbd "k") 'ros-helm/kill-ros-node)
    (define-key map (kbd "c") 'ros-helm/interrupt-ros-node)
    (define-key map (kbd "q") (lambda () (interactive) (delete-window)))
    map)
  "Keymap for the ros-node major mode")

(defun ros-helm/interrupt-ros-node ()
  (interactive)
  (let ((ros-node-process (get-buffer-process (current-buffer))))
    (interrupt-process ros-node-process)))

(defun ros-helm/kill-ros-node ()
  (interactive)
  (let ((ros-node-process (get-buffer-process (current-buffer))))
    (kill-process ros-node-process)))

(defun ros-process-filter (process string)
  "Applies `xterm-color-filter' to the text before outputting it
to the process buffer."
  (when (buffer-live-p (process-buffer process))
    (with-current-buffer (process-buffer process)
      (let ((moving (= (point) (process-mark process))))
        (save-excursion
          (message "FILTER")
          (goto-char (process-mark process))
          (insert (xterm-color-filter string))
          (set-marker (process-mark process) (point)))
        (if moving (goto-char (process-mark process)))))))

(define-derived-mode ros-process-mode fundamental-mode "ROS Node Mode"
  "Major mode for handling a rosrun console."
  (set-process-filter (get-buffer-process (current-buffer)) 'ros-process-filter))

(provide 'ros-process-mode)
