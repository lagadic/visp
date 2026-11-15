function openTab(evt, tabName) {
    var i, tabcontent, tablinks;
    var container = evt.currentTarget.closest('.tab-container');

    tabcontent = container.getElementsByClassName("tab-content");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }

    tablinks = container.getElementsByClassName("tab-btn");
    for (i = 0; i < tablinks.length; i++) {
      tablinks[i].className = tablinks[i].className.replace(" active", "");
    }

    container.querySelector('[data-tab="' + tabName + '"]').style.display = "block";
    evt.currentTarget.className += " active";
  }
