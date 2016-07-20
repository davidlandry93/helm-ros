

(defvar helm-source-ros-services
  (helm-build-async-source "Services"
    :candidates-process (lambda ()
                          (start-process-shell-command
                           "find-services" nil
                           "find"  "/opt/ros/indigo/" "|" "grep" "\\.srv$"))))

(defvar launchfile-candidate-list-cache nil)

(defun fetch-launchfile-candidate-list ()
  "Fills the `launchfile-candidate-list-cache' variable with the name of the launchfiles we could find"
  (with-temp-buffer
    (call-process-shell-command
     "find /opt/ros/indigo -type f -name \"*.launch\"" nil t)
    (goto-char (point-min))
    (let (launchfiles)
      (while (not (eq 0 (count-lines (point) (point-max))))
        (push (buffer-substring (point) (line-end-position)) launchfiles)
        (forward-line))
      (setq launchfile-candidate-list-cache launchfiles)
      (kill-buffer))))

(defun launchfile-candidate-list ()
  (if launchfile-candidate-list-cache
      launchfile-candidate-list-cache
    (setq launchfile-candidate-list-cache
          (accept-process-output
           (call-process-shell-command
            "find-launchfiles" nil
            "find" "/opt/ros/indigo/" "|" "grep" "\\.launch$")))))

(defvar helm-source-ros-launchfiles
  (helm-build-async-source "Launchfiles"
    :candidates-process (lambda ()
                          )
    :action ("Open file" . (lambda (launchfile-str)
                             (interactive)
                             (find-file launchfile-str)))))

;;;###autoload
(defun ros-helm ()
  (interactive)
  (helm :sources '(helm-source-ros-services helm-source-ros-launchfiles)
        :buffer "*ros-helm*"))

(provide 'ros-helm)
