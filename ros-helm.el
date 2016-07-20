

(defvar helm-source-ros-services
  (helm-build-async-source "Services"
    :candidates-process (lambda ()
                          (start-process-shell-command
                           "find-services" nil
                           "find"  "/opt/ros/indigo/" "|" "grep" "\\.srv$"))))

(defvar launchfile-candidate-list-cache nil)

(defun fetch-launchfile-candidate-list ()
  "Fills the `launchfile-candidate-list-cache' variable with the name of the launchfiles we could find, an then returns it."
  (with-current-buffer "    *temp*"
    (call-process-shell-command
     "find /opt/ros/indigo -type f -name \"*.launch\"" nil (get-buffer "    *temp*"))
    (goto-char (point-min))
    (let (launchfiles)
      (while (not (eq 0 (count-lines (point) (point-max))))
        (push (buffer-substring (line-beginning-position) (point)) launchfiles)
        (message (buffer-substring (line-beginning-position) (point)))
        (end-of-line))
      (setq launchfile-candidate-list-cache launchfiles)
      (kill-buffer)))
  launchfile-candidate-list-cache)

(defun launchfile-candidate-list ()
  (if launchfile-candidate-list-cache
      (fetch-launchfile-candidate-list)
    launchfile-candidate-list-cache))

(defvar helm-source-ros-launchfiles
  (helm-build-sync-source "Launchfiles"
    :candidates (map 'list (lambda (path) (make-symbol path)) (launchfile-candidate-list))
    :action '(("Open file" . (lambda (launchfile-str)
                             (interactive)
                             (find-file launchfile-str))))))

(defun ros-helm ()
  (interactive)
  (helm :sources '(helm-source-ros-services helm-source-ros-launchfiles)
        :buffer "*ros-helm*"))

(provide 'ros-helm)
