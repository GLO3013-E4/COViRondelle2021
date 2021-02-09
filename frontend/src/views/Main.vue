<template>
  <v-container>
    <div class="d-flex justify-center mt-5">
      <h1 class="font-weight-medium">Robot culinaire</h1>
    </div>
    <div class="d-flex justify-center mt-5">
<<<<<<< HEAD
      <RobotInformation />
=======
      <main-layout />
>>>>>>> develop
    </div>
  </v-container>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import RobotInformation from "../layouts/MainLayout.vue";

@Component({
<<<<<<< HEAD
  components: { RobotInformation: RobotInformation }
=======
  components: { MainLayout }
>>>>>>> develop
})
export default class Main extends Vue {}
</script>

<style></style>
