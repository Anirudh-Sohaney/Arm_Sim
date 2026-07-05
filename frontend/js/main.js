/* main.js — Bootstrap: wire everything together and connect. */
(function () {
  document.getElementById("error-dismiss").onclick = () => {
    document.getElementById("error-banner").classList.add("hidden");
  };

  WSClient.connect();
})();
