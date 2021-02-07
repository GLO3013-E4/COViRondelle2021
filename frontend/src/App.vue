<template>
  <v-app>
    <MainView />
  </v-app>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import MainView from "./views/Main.vue";

@Component({
  components: {
    MainView
  }
})
export default class App extends Vue {}
</script>

<style></style>
