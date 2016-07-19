
;;;###autoload
(defun ros-helm ()
  (interactive)
  (helm :sources '(helm-source-ros-services helm-source-ros-launchfiles)
        :buffer "*ros-helm*"))

(defvar helm-source-ros-services
  (helm-build-async-source "Services"
    :candidates-process (lambda ()
                          (start-process-shell-command "find-services" nil "find"  "/opt/ros/indigo/" "|" "grep" "\\.srv$"))))

(defvar helm-source-ros-launchfiles
  (helm-build-async-source "Launchfiles"
    :candidates-process (lambda ()
                          (start-process-shell-command "find-launchfiles" nil "find" "/opt/ros/indigo/" "|" "grep" "\\.launch"))))

(provide 'ros-helm)
